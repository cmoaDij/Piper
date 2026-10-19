#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dmrobotics offline H5 playback demo (Flux remote processing)

What this script does
---------------------
- Loads tactile frames from an H5 file (test.h5) using dmrobotics.utils.read_h5
- Uses the first frame as the base frame (setBaseFrame)
- For each frame:
  - Runs remote processing: infer image + deformation + depth + shear
  - Visualizes outputs with OpenCV windows:
    - img (infer image)
    - deformation (arrows)
    - shear (arrows)
    - depth (scaled to uint8 for preview)
- Prints approximate output FPS to the terminal

Keyboard controls
-----------------
- 'q' : quit

Notes
-----
- This script is intended for interactive visualization (requires a display).
- If you run on a headless device, remove cv2.imshow/cv2.waitKey and replace with
  terminal polling + saving results to disk/UDP/etc.
"""

import time
import cv2
import numpy as np

from dmrobotics import Sensor, SensorOptions, Mode
from dmrobotics.utils import put_arrows_on_image, read_h5


if __name__ == "__main__":
    # Read metadata and the first frame from H5
    dev_id, max_id, dmimg = read_h5("test.h5", 0)

    opt = SensorOptions(
        dev_id=dev_id,                 # device index or serial, depending on your deployment
        backend="cpu",                # key: use remote Flux backend
        mode=Mode.STANDARD,
        show_fps=False,
        enable_deformation=False,
        enable_depth=False,
        enable_shear=False,
        enable_force=False,
        remote_addr="192.168.127.10:50051",
        pc_host="192.168.127.100",
        pc_port=60000,
    )

    sensor = Sensor(opt)

    frame_num = 0.0
    start_time = time.time()

    # Canvas for arrow visualization (match your expected H/W)
    black_img = np.zeros((288, 384, 3), dtype=np.uint8)

    i = 0
    while True:
        _, max_id, dmimg = read_h5("test.h5", i)
        if i > max_id:
            break

        # Set base frame using the first frame
        if i == 0:
            sensor.setBaseFrame(dmimg)

        # Run remote processing on the current frame
        img, deformation, depth, shear,force,distforce = sensor.process(dmimg, getdepth=True, getshear=True)
        if img is None:
            break

        # Visualization
        cv2.imshow("img", img)
        cv2.imshow("deformation", put_arrows_on_image(black_img, deformation * 50))
        cv2.imshow("shear", put_arrows_on_image(black_img, shear * 50))

        depth_img = (depth * 50).clip(0, 255).astype(np.uint8)
        cv2.imshow("depth", depth_img)

        # Keyboard
        k = cv2.waitKey(1)
        frame_num += 1

        if (k & 0xFF) == ord("q"):
            break

        # FPS reporting (rough)
        if time.time() - start_time > 1.0:
            fps = frame_num / (time.time() - start_time)
            print(f"Output FPS is: {fps:.2f}", end="\r")
            frame_num = 0.0
            start_time = time.time()

        i += 1

    sensor.disconnect()
    cv2.destroyAllWindows()
