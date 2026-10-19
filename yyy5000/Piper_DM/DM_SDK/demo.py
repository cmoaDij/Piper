#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dmrobotics_demo_viewer.py

Purpose
-------
A minimal interactive viewer for the dmrobotics tactile sensor pipeline.

What it does
------------
- Connects to ONE sensor (device index or serial/suffix).
- Waits for new frames (by frame id).
- Fetches and visualizes:
  - Infer image (always)
  - (Optional) Raw image
  - (Optional) Deformation (flow -> arrows)
  - (Optional) Shear (curl -> arrows)
  - (Optional) Depth (float -> 8-bit preview)

Keyboard controls
-----------------
q : quit
r : reset (keep no-contact during reset)
1 : toggle raw
2 : toggle depth
3 : toggle shear
+ / = : increase max FPS limit by 10
- / _ : decrease max FPS limit by 10

Notes
-----
- This demo assumes ALL getters return (fid, data):
    getInferImg()      -> (fid, DmTacImage)
    getRawImg()        -> (fid, DmTacImage)
    getDeformation2D() -> (fid, np.ndarray HxWx2 float32)
    getShear()         -> (fid, np.ndarray HxWx2 float32)
    getDepth()         -> (fid, np.ndarray HxW   float32)
- Device status is an int (your design): 0 = OK, non-zero = not-ready/resetting/disconnected.
"""

import time
import argparse
import cv2
import numpy as np

from dmrobotics import Sensor, SensorOptions, Mode
from dmrobotics.utils import  put_arrows_on_image

def depth_to_u8(depth: np.ndarray, scale: float = 100.0) -> np.ndarray:
    """Map float depth to uint8 for quick visualization (demo-only)."""
    if depth is None:
        return None
    d = np.asarray(depth)
    if d.size == 0:
        return None
    return (d * scale).clip(0, 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev_id", type=str, default="0", help="Device index or serial")
    ap.add_argument("--backend", type=str, default="cuda", help="Backend name (cpu/cuda/Flux/...)")
    ap.add_argument("--mode", type=str, default="standard", choices=["standard", "high"])
    ap.add_argument("--max_fps", type=int, default=120)
    ap.add_argument("--show_fps", action="store_true")

    # If none provided, default: def+depth+shear ON, raw OFF
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--defm", action="store_true")
    ap.add_argument("--depth", action="store_true")
    ap.add_argument("--shear", action="store_true")

    # Remote args (ignored if backend does not use them)
    ap.add_argument("--remote_addr", type=str, default="192.168.127.10:50051")
    ap.add_argument("--pc_host", type=str, default="192.168.127.100")
    ap.add_argument("--pc_port", type=int, default=60001)

    args = ap.parse_args()

    if not (args.raw or args.defm or args.depth or args.shear):
        args.raw = False
        args.defm = True
        args.depth = True
        args.shear = True

    # dev_id can be int (index) or string (serial/suffix)
    try:
        dev_id = int(args.dev_id)
    except Exception:
        dev_id = args.dev_id

    mode = Mode.HIGH if args.mode.lower() == "high" else Mode.STANDARD

    opt = SensorOptions(
        dev_id=dev_id,
        backend=args.backend,
        mode=mode,
        show_fps=args.show_fps,
        max_fps=args.max_fps,
        enable_raw=args.raw,
        enable_deformation=args.defm,
        enable_depth=args.depth,
        enable_shear=args.shear,
        enable_force=False,
        remote_addr=args.remote_addr,
        pc_host=args.pc_host,
        pc_port=args.pc_port,
    )
    sensor = Sensor(opt)

    # Create windows once (toggling only affects fetching/streaming, not window creation)
    cv2.namedWindow("infer", cv2.WINDOW_NORMAL)
    cv2.namedWindow("raw", cv2.WINDOW_NORMAL)
    cv2.namedWindow("deformation", cv2.WINDOW_NORMAL)
    cv2.namedWindow("shear", cv2.WINDOW_NORMAL)
    cv2.namedWindow("depth", cv2.WINDOW_NORMAL)

    canvas_def = None
    canvas_shr = None

    last_fid = -1
    loop_cnt = 0
    t0 = time.time()

    try:
        while True:
            # Device state: 0 = OK, non-zero = not ready
            st = sensor.getDevStatus()
            if st != 0:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    print("[Action] reset()")
                    sensor.reset()
                time.sleep(0.01)
                continue

            # Wait for a new frame (by last_fid)
            if not sensor.wait_for_new(last_fid, timeout_ms=500):
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    print("[Action] reset()")
                    sensor.reset()
                continue

            fid_for_loop = last_fid

            # Infer (always fetch/display)
            fid, inf = sensor.getInferImg()
            if inf is not None and getattr(inf, "img", None) is not None:
                cv2.imshow("infer", inf.img)
                fid_for_loop = fid

            # Raw
            if args.raw:
                fid, raw = sensor.getRawImg()
                if raw is not None and getattr(raw, "img", None) is not None:
                    cv2.imshow("raw", raw.img)
                    fid_for_loop = fid

            # Deformation (flow)
            if args.defm:
                fid, flow = sensor.getDeformation2D()
                if flow is not None:
                    h, w = flow.shape[:2]
                    if canvas_def is None or canvas_def.shape[:2] != (h, w):
                        canvas_def = np.zeros((h, w, 3), np.uint8)
                    else:
                        canvas_def.fill(0)
                    cv2.imshow("deformation", put_arrows_on_image(canvas_def, flow, step=16, scale=20.0))
                    fid_for_loop = fid

            # Shear 
            if args.shear:
                fid, shr = sensor.getShear()
                if shr is not None:
                    h, w = shr.shape[:2]
                    if canvas_shr is None or canvas_shr.shape[:2] != (h, w):
                        canvas_shr = np.zeros((h, w, 3), np.uint8)
                    else:
                        canvas_shr.fill(0)
                    cv2.imshow("shear", put_arrows_on_image(canvas_shr, shr, step=16, scale=20.0))
                    fid_for_loop = fid

            # Depth
            if args.depth:
                fid, dep = sensor.getDepth()
                if dep is not None:
                    cv2.imshow("depth", depth_to_u8(dep, scale=100.0))
                    fid_for_loop = fid

            # ---- Required keyboard features ----
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                print("[Action] reset()")
                sensor.reset()
            elif key == ord("1"):
                args.raw = not args.raw
                print("[Toggle] raw =", args.raw)
                sensor.setEnableFlags(raw=args.raw,
                                      deformation=args.defm,
                                      depth=args.depth,
                                      shear=args.shear)
            elif key == ord("2"):
                args.depth = not args.depth
                print("[Toggle] depth =", args.depth)
                sensor.setEnableFlags(raw=args.raw,
                                      deformation=args.defm,
                                      depth=args.depth,
                                      shear=args.shear)
            elif key == ord("3"):
                args.shear = not args.shear
                print("[Toggle] shear =", args.shear)
                sensor.setEnableFlags(raw=args.raw,
                                      deformation=args.defm,
                                      depth=args.depth,
                                      shear=args.shear)
            elif key == ord("+") or key == ord("="):
                args.max_fps = min(240, args.max_fps + 10)
                print("[FPS] setMaxFPS =", args.max_fps)
                sensor.setMaxFPS(args.max_fps)
            elif key == ord("-") or key == ord("_"):
                args.max_fps = max(1, args.max_fps - 10)
                print("[FPS] setMaxFPS =", args.max_fps)
                sensor.setMaxFPS(args.max_fps)

            last_fid = fid_for_loop

            # Demo loop FPS (how fast THIS script fetches + shows)
            loop_cnt += 1
            now = time.time()
            if now - t0 >= 1.0:
                print(f"[Main] loop FPS: {loop_cnt / (now - t0):.2f}")
                loop_cnt = 0
                t0 = now

    finally:
        try:
            sensor.disconnect()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
