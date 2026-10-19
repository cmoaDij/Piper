#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dmrobotics H5 recorder demo (capture raw frames and save to an H5 file)

What this script does
---------------------
- Connects to a dmrobotics sensor service (local/remote depending on backend/remote_addr)
- Continuously waits for new frames and reads the raw image (DmTacImage)
- Displays the raw image in an OpenCV window
- Records the raw frames into an H5 file ("test.h5"):
  - The first frame initializes the file (init_h5)
  - Subsequent frames are appended (append_h5)

Keyboard controls
-----------------
- 'q' : quit
- 'r' : set the current raw frame as the base frame on the remote sensor

Notes
-----
- This script uses OpenCV windows (cv2.imshow / cv2.waitKey), so it requires a display.
"""

import time
import cv2
import numpy as np

from dmrobotics import (
    Sensor,
    SensorOptions,
    Mode
)

from dmrobotics.utils import init_h5, append_h5


def main():
    opt = SensorOptions(
        dev_id=4,                      # device index or serial suffix (depends on your deployment)
        backend="cpu",                 # processing backend (your comment says remote; keep as-is)
        mode=Mode.STANDARD,
        show_fps=True,
        enable_deformation=False,
        enable_depth=False,
        enable_shear=False,
        remote_addr="10.42.0.224:50052",
        pc_host="10.42.0.1",
        pc_port=60000,
    )

    sensor = Sensor(opt)

    last_fid = -1
    frame_cnt = 0
    t0 = time.time()

    init_flag = True

    try:
        while True:
            # Device status: 0=OK, 1=RESETTING, 2=DISCONNECTED
            if sensor.getDevStatus() != 0:
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                time.sleep(0.01)
                continue

            # Wait for a new frame
            got_new = sensor.wait_for_new(last_fid, timeout_ms=500)
            if not got_new:
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                continue

            # Read raw frame
            fid, raw = sensor.getRawImg()
            last_fid = fid

            # Display raw image
            if raw is not None and getattr(raw, "img", None) is not None:
                cv2.imshow("img", raw.img)

            # Record to H5
            if init_flag:
                init_h5("test.h5", raw)
                init_flag = False
            else:
                append_h5("test.h5", raw)

            # Hotkeys
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("r"):
                # Use current raw frame as the base frame
                if raw is not None:
                    sensor.setBaseFrame(raw)
                    print("Set base frame on remote sensor")

            # FPS stats
            frame_cnt += 1
            now = time.time()
            if now - t0 >= 1.0:
                fps = frame_cnt / (now - t0)
                print(f"Display FPS: {fps:.2f}")
                frame_cnt = 0
                t0 = now

    finally:
        sensor.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
