# This should be done in OpenMV IDE

import sensor
import image
import time
import struct
from pyb import SPI


# Camera

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)   # 160 x 120
sensor.skip_frames(time=2000)
# this threshold is for like a bright red object for inside lighting, if you go to machine vision you can edit the threshold, so the lighting would be a little dfferent outside and i recommend editing it accordingly
red_threshold = (10, 100, 30, 127, -40, 80)


# SPI


spi = SPI(
    2,
    spi.slave,
    polarity=0,
    phase=0
)

# Keep this below the Pi's 4096-byte limit.
chunk_size = 1000

clock = time.clock()


while True:

    clock.tick()

    img = sensor.snapshot()

  
    # Detect red

    blobs = img.find_blobs(
        [red_threshold],
        pixels_threshold=20,
        area_threshold=20,
        merge=True
    )

    cx = -1
    cy = -1
    bw = 0
    bh = 0

    if blobs:

        blob = blobs[0]

        cx = blob.cx
        cy = blob.cy
        bw = blob.w
        bh = blob.h

        img.draw_rectangle(blob.rect)


    # Compress JPEG


    jpeg = img.compress(
        quality=40
    ).bytearray()

    jpeg_len = len(jpeg)

    print(
        "FRAME:",
        cx,
        cy,
        bw,
        bh,
        "JPEG:",
        jpeg_len
    )

    
    # Number of chunks
    

    total_chunks = (
        jpeg_len + CHUNK_SIZE - 1
    ) // CHUNK_SIZE

  
    # Send each chunk


    for chunk_num in range(total_chunks):

        start = chunk_num * chunk_size

        end = min(
            start + chunk_size,
            jpeg_len
        )

        chunk = jpeg[start:end]

        actual_len = len(chunk)

    

        header = struct.pack(
            "<HhhhhIHHH",
            0x55AA,
            cx,
            cy,
            bw,
            bh,
            jpeg_len,
            chunk_num,
            total_chunks,
            actual_len
        )

        packet = bytearray()

        packet.extend(header)

        packet.extend(chunk)

      

        while len(packet) < (
            20 + chunk_size
        ):

            packet.append(0)

        try:

            spi.send(
                packet,
                timeout=2000
            )

        except Exception as e:

            print(
                "spi error :(",
                e
            )

            break

    
