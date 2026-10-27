#This is the raspberry pi code, i ran it in command prompt
#make sure you did spi connection right with your raspberry pi and camera, you can google the wiring :)

import struct
import threading
import time

import spidev
from flask import Flask, Response
from pymavlink import mavutil

# general settings

#how the camera and pi communicate
spi_bus = 0
spi_device = 0
#the speed can be edited, this amount was a little laggy when tested in the air, but i recommend slowly increasing it bc spi timing can get weird if you increase too much
spi_speed = 25_000_000

chunk_size = 1000
header_size = 20
packet_size = header_size + chunk_size

frame_width = 160
frame_height = 120

# The camera sends blob.cx / blob.cy, which are the size/coordinates of the boxed red object
blob_is_corner = False

center_threshold = 12        
yaw_gain = 0.004             
max_yaw_rate = 0.5           
forward_velocity = -0.5       # in m/s, idk why but it has to be negative to go forward, if it go backwads then switch it, i might jsut not know where the front of the drone was

control_rate_hz = 10         # setpoint stream rate (ArduPilot wants >= 2 Hz)
detection_timeout = 0.5      # s without a blob before hovering
require_guided = True        # don't fight the pilot in other modes

# make dry run true if you want it to send 0 commands to FC for testing
dry_run = False




min_tracking_fps = 8.0

# pull the id of your FC and run the command when you hook it up to your FC to figure it out
mavlink_port = (
    "/dev/serial/by-id/"
    "usb-ArduPilot_microBlue_3D003F001451333239363937-if00"
)
#go into parameters and the port you use to figure out its baud rate
mavlink_baud = 115200

#this is a bitfield, so it tells the pi what parameters  the drone sends to ignore
type_mask_vel_yawrate = 0b0000011111000111



state_lock = threading.Lock()

latest_frame = None
detection = {"cx": 0, "cy": 0, "valid": False, "t": 0.0}
measured_fps = 0.0

# SPI run

spi = spidev.SpiDev()
spi.open(spi_bus, spi_device)
spi.max_speed_hz = spi_speed
spi.mode = 0

# mavlink connection

master = mavutil.mavlink_connection(
    mavlink_port,
    baud=mavlink_baud,
    source_system=250,
    source_component=191,
)

print("Waiting for FC heartbeat...")
master.wait_heartbeat()
print(
    "MAVLink connected system",
    master.target_system,
    "component",
    master.target_component,
)

target_system = master.target_system
target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1


def mavlink_reader():
    """Drain the link so master.flightmode stays current."""
    while True:
        try:
            master.recv_match(blocking=True, timeout=1)
        except Exception as e:
            print("MAVLINK READ ERROR:", e)
            time.sleep(0.5)


def heartbeat_sender():
    """Announce the Pi at 1 Hz
    yay then its connected
    """
    while True:
        try:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception as e:
            print("HEARTBEAT ERROR:", e)
        time.sleep(1.0)


def send_velocity_body(vx, vy, vz, yaw_rate):
    """Body-frame velocity setpoint. All zeros == hold position."""
    if dry_run:
        if yaw_rate > 0.001:
            direction = "-> RIGHT (clockwise)"
        elif yaw_rate < -0.001:
            direction = "<- LEFT (counter-clockwise)"
        else:
            direction = "   hold"
        print(f"  [DRY RUN] vx={vx:+.2f} yaw_rate={yaw_rate:+.3f} {direction}")
        return

    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask_vel_yawrate,
        0.0, 0.0, 0.0,          # x, y, z      (ignored)
        vx, vy, vz,             # velocity
        0.0, 0.0, 0.0,          # accel        (ignored)
        0.0,                    # yaw          (ignored)
        yaw_rate,
    )


def hover():
    send_velocity_body(0.0, 0.0, 0.0, 0.0)


#  control loop

def tracking_command(cx, cy):
    """Return (forward_velocity, yaw_rate) for a blob centre at cx, cy."""
    error_x = cx - (frame_width // 2)

    if abs(error_x) <= center_threshold:
        yaw_rate = 0.0
    else:
        yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, error_x * yaw_gain))

    if dry_run:
        side = "RIGHT" if error_x > 0 else "LEFT" if error_x < 0 else "CENTER"
        print(f"blob cx={cx:3d} err={error_x:+4d} (object is {side})")

    return forward_velocity, yaw_rate


def control_loop():
    period = 1.0 / control_rate_hz
    last_state = None

    while True:
        loop_start = time.monotonic()

        with state_lock:
            d = dict(detection)
            fps = measured_fps

        fresh = d["valid"] and (time.monotonic() - d["t"]) < detection_timeout
        fast_enough = fps >= min_tracking_fps

        if require_guided and not dry_run and master.flightmode != "GUIDED":
            if last_state != "not_guided":
                print("Mode is", master.flightmode, "-- not sending setpoints")
                last_state = "not_guided"
            time.sleep(period)
            continue

        try:
            if fresh and fast_enough:
                vx, yaw_rate = tracking_command(d["cx"], d["cy"])
                send_velocity_body(vx, 0.0, 0.0, yaw_rate)
                if last_state != "tracking":
                    print("TRACKING")
                    last_state = "tracking"
            elif fresh and not fast_enough:
                hover()
                if last_state != "too_slow":
                    print(
                        f"TARGET SEEN but only {fps:.1f} fps "
                        f"(need {min_tracking_fps:.0f}) -- HOVERING"
                    )
                    last_state = "too_slow"
            else:
                hover()
                if last_state != "hover":
                    print("NO TARGET -- HOVERING")
                    last_state = "hover"
        except Exception as e:
            print("SETPOINT ERROR:", e)

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, period - elapsed))


# SPI receiver code
#its a low level protcol, so basically the bytes that the camera stream sends have to be manually fomatted 
#magic bytes identify commands, and chunks manage payload sizes  from the camera, but hey, magic is a pretty cool name, nice work software people

def receive_frames():
    global latest_frame
    global measured_fps

    current_frame = None
    expected_chunk = 0
    total_chunks_expected = 0


    f_cx = f_cy = f_bw = f_bh = 0

    stats = {"good": 0, "magic": 0, "chunk": 0, "jpeg": 0, "len": 0}
    last_report = time.monotonic()

    while True:
        try:
            data = bytes(spi.xfer2([0] * packet_size))

            now = time.monotonic()
            if now - last_report >= 2.0:
                span = now - last_report
                fps = stats["good"] / span
                with state_lock:
                    measured_fps = fps
                print(
                    f"[{fps:.1f} fps] "
                    f"good={stats['good']} badmagic={stats['magic']} "
                    f"chunkerr={stats['chunk']} badjpeg={stats['jpeg']} "
                    f"badlen={stats['len']}"
                )
                for k in stats:
                    stats[k] = 0
                last_report = now

            (
                magic, cx, cy, bw, bh,
                jpeg_len, chunk_num, total_chunks, chunk_len,
            ) = struct.unpack("<HhhhhIHHH", data[:header_size])

            if magic != 0x55AA:
                stats["magic"] += 1
                continue

            if chunk_len > chunk_size:
                stats["len"] += 1
                continue

            if chunk_num == 0:
                current_frame = bytearray()
                expected_chunk = 0
                total_chunks_expected = total_chunks
                f_cx, f_cy, f_bw, f_bh = cx, cy, bw, bh

            if chunk_num != expected_chunk:
                stats["chunk"] += 1
                current_frame = None
                continue

            if current_frame is None:
                continue

            current_frame.extend(data[header_size:header_size + chunk_len])
            expected_chunk += 1

            if chunk_num + 1 != total_chunks_expected:
                continue

            jpeg = bytes(current_frame)
            current_frame = None

            if not (len(jpeg) >= 2 and jpeg[0] == 0xFF and jpeg[1] == 0xD8):
                stats["jpeg"] += 1
                continue

            stats["good"] += 1

     
            has_blob = f_bw > 0 and f_bh > 0 and f_cx >= 0 and f_cy >= 0

            if has_blob and blob_is_corner:
                center_x = f_cx + f_bw // 2
                center_y = f_cy + f_bh // 2
            else:
                center_x, center_y = f_cx, f_cy

            with state_lock:
                latest_frame = jpeg
                if has_blob:
                    detection.update(
                        cx=center_x,
                        cy=center_y,
                        valid=True,
                        t=time.monotonic(),
                    )
                else:
                    detection["valid"] = False

        except Exception as e:
            print("SPI ERROR:", e)
            time.sleep(1)


# flask camera stream, this code streams the camera feed on your computer through a web browser link that it generates

app = Flask(__name__)


def generate_frames():
    while True:
        with state_lock:
            frame = latest_frame
        if frame is not None:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
        time.sleep(0.1)


@app.route("/")
def index():
    return (
        "<!DOCTYPE html><html><head><title>Camera</title></head><body>"
        "<h1>Super Amazing Camera Stream</h1>"
        '<img src="/video" width="640" height="480">'
        "</body></html>"
    )


@app.route("/video")
def video():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


#  if everything is good it will generate the link and stream
#watch the magic happen, electronics are so fun

if __name__ == "__main__":
    for target in (mavlink_reader, heartbeat_sender, receive_frames, control_loop):
        threading.Thread(target=target, daemon=True).start()

    print("http://<PI-IP>:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)
#random suggestion,  make the target hot pink, i think that would be super fun, or any non natural occuring color,and don't wear that color when testing or its gonna chase you, great joke tho if you set the speed to really low :)
