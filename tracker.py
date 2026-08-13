#!/usr/bin/env python3

import struct
import threading
import time

import spidev
from flask import Flask, Response
from pymavlink import mavutil

# ------------------------------------------------------------------ settings

SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 25_000_000

CHUNK_SIZE = 1000
HEADER_SIZE = 20
PACKET_SIZE = HEADER_SIZE + CHUNK_SIZE

FRAME_WIDTH = 160
FRAME_HEIGHT = 120

# The camera sends blob.cx / blob.cy, which are already the blob's CENTRE.
BLOB_IS_CORNER = False

CENTER_THRESHOLD = 12        # px dead-band, no command inside this
YAW_GAIN = 0.004             # rad/s per px of error -- start small
MAX_YAW_RATE = 0.5           # rad/s clamp
FORWARD_VELOCITY = -0.5       # m/s -- leave at 0 until yaw tracking is proven

CONTROL_RATE_HZ = 10         # setpoint stream rate (ArduPilot wants >= 2 Hz)
DETECTION_TIMEOUT = 0.5      # s without a blob before hovering
REQUIRE_GUIDED = True        # don't fight the pilot in other modes

# make dry run true if you want it to send 0 commands to FC for testing
DRY_RUN = False



# Hover instead of tracking until OpenMV side is fixed.
MIN_TRACKING_FPS = 8.0

MAVLINK_PORT = (
    "/dev/serial/by-id/"
    "usb-ArduPilot_microBlue_3D003F001451333239363937-if00"
)
MAVLINK_BAUD = 115200

# bits 0-2  ignore position
# bits 3-5  CLEAR -> use velocity
# bits 6-8  ignore acceleration
# bit  10   ignore yaw angle
# bit  11   CLEAR -> use yaw_rate   <-- this was set in your original mask
TYPE_MASK_VEL_YAWRATE = 0b0000011111000111

# -------------------------------------------------------------------- state

state_lock = threading.Lock()

latest_frame = None
detection = {"cx": 0, "cy": 0, "valid": False, "t": 0.0}
measured_fps = 0.0

# ---------------------------------------------------------------------- SPI

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = 0

# ------------------------------------------------------------------ MAVLink

master = mavutil.mavlink_connection(
    MAVLINK_PORT,
    baud=MAVLINK_BAUD,
    source_system=250,
    source_component=191,
)

print("Waiting for FC heartbeat...")
master.wait_heartbeat()
print(
    "MAVLink connected -- system",
    master.target_system,
    "component",
    master.target_component,
)

TARGET_SYSTEM = master.target_system
TARGET_COMPONENT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1


def mavlink_reader():
    """Drain the link so master.flightmode stays current."""
    while True:
        try:
            master.recv_match(blocking=True, timeout=1)
        except Exception as e:
            print("MAVLINK READ ERROR:", e)
            time.sleep(0.5)


def heartbeat_sender():
    """Announce the Pi at 1 Hz.

    Identifies as an onboard controller, not a GCS. If FS_GCS_ENABLE is on
    and you want the Pi to satisfy that failsafe, set SYSID_MYGCS on the FC
    to match this connection's source_system (250) -- do NOT set the Pi to
    255, which Mission Planner already uses.
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
    if DRY_RUN:
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
        TARGET_SYSTEM,
        TARGET_COMPONENT,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        TYPE_MASK_VEL_YAWRATE,
        0.0, 0.0, 0.0,          # x, y, z      (ignored)
        vx, vy, vz,             # velocity
        0.0, 0.0, 0.0,          # accel        (ignored)
        0.0,                    # yaw          (ignored)
        yaw_rate,
    )


def hover():
    send_velocity_body(0.0, 0.0, 0.0, 0.0)


# ------------------------------------------------------------- control loop

def tracking_command(cx, cy):
    """Return (forward_velocity, yaw_rate) for a blob centre at cx, cy."""
    error_x = cx - (FRAME_WIDTH // 2)

    if abs(error_x) <= CENTER_THRESHOLD:
        yaw_rate = 0.0
    else:
        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, error_x * YAW_GAIN))

    if DRY_RUN:
        side = "RIGHT" if error_x > 0 else "LEFT" if error_x < 0 else "CENTER"
        print(f"blob cx={cx:3d} err={error_x:+4d} (object is {side})")

    return FORWARD_VELOCITY, yaw_rate


def control_loop():
    period = 1.0 / CONTROL_RATE_HZ
    last_state = None

    while True:
        loop_start = time.monotonic()

        with state_lock:
            d = dict(detection)
            fps = measured_fps

        fresh = d["valid"] and (time.monotonic() - d["t"]) < DETECTION_TIMEOUT
        fast_enough = fps >= MIN_TRACKING_FPS

        if REQUIRE_GUIDED and not DRY_RUN and master.flightmode != "GUIDED":
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
                        f"(need {MIN_TRACKING_FPS:.0f}) -- HOVERING"
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


# ------------------------------------------------------------ SPI receiver

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
            data = bytes(spi.xfer2([0] * PACKET_SIZE))

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
            ) = struct.unpack("<HhhhhIHHH", data[:HEADER_SIZE])

            if magic != 0x55AA:
                stats["magic"] += 1
                continue

            if chunk_len > CHUNK_SIZE:
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

            current_frame.extend(data[HEADER_SIZE:HEADER_SIZE + chunk_len])
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

            if has_blob and BLOB_IS_CORNER:
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


# -------------------------------------------------------------------- Flask

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
        "<h1>Camera Stream</h1>"
        '<img src="/video" width="640" height="480">'
        "</body></html>"
    )


@app.route("/video")
def video():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# --------------------------------------------------------------------- main

if __name__ == "__main__":
    for target in (mavlink_reader, heartbeat_sender, receive_frames, control_loop):
        threading.Thread(target=target, daemon=True).start()

    print("http://<PI-IP>:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)
