#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dmrobotics visualization demo (Windows single-process / Linux multi-process)

What this script does
---------------------
- Connects to a dmrobotics tactile sensor (local or remote via gRPC)
- Continuously reads:
  - infer image (for display)
  - deformation (2D flow, arrow visualization)
  - shear (2D curl, arrow visualization)
  - depth (float map, scaled to uint8 for display)
- On Windows:
  - Runs everything in one process (simpler and more stable for OpenCV windows)
- On non-Windows platforms (Linux, e.g., PC host):
  - Uses a separate visualization process to render deformation/shear windows
  - Main process handles sensor I/O and sends frames via a small queue

Keyboard controls
-----------------
- 'q' : quit
- 'r' : reset the sensor (request base reset / re-initialize)
  - In the visualizer process, 'r' requests a reset via an IPC event.

Notes
-----
- The queue is intentionally small to avoid backlog and slow shutdown.
- This demo is designed for interactive visualization; for headless usage,
  replace cv2.waitKey() with terminal key polling.
"""

import time
import multiprocessing as mp
import queue  # used to catch queue.Full exceptions
import cv2
import numpy as np
import os

from dmrobotics import (
    Sensor,
    SensorOptions,
    Mode,
)

from dmrobotics.utils import put_arrows_on_image

IS_WINDOWS = (os.name == "nt")


def visualizer_process(queue_in: mp.Queue, running_event, reset_event):
    """
    Visualization process (non-Windows only).

    Receives frames from the main process:
      - ("deformation", flow) -> draw arrows
      - ("shear", curl)       -> draw arrows
      - ("infer", image)      -> show directly (optional)

    Hotkeys (handled inside the OpenCV window loop):
      - 'q' : stop everything
      - 'r' : request sensor reset (sets reset_event)
    """
    vis_canvas = None
    win_def = "deformation"
    win_shear = "shear"
    win_infer = "infer"

    try:
        while running_event.is_set():
            try:
                data = queue_in.get(timeout=0.05)
            except Exception:
                # Also poll keys even if no new frame arrives
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    print("[Vis] 'q' pressed. Signal stopping...")
                    running_event.clear()
                    break
                elif key == ord("r"):
                    print("[Vis] 'r' pressed in vis window. Request reset...")
                    reset_event.set()
                continue

            img_type, img_data = data

            if img_type == "deformation":
                h, w = img_data.shape[:2]
                if vis_canvas is None or vis_canvas.shape[:2] != (h, w):
                    vis_canvas = np.zeros((h, w, 3), dtype=np.uint8)
                else:
                    vis_canvas.fill(0)

                vis = put_arrows_on_image(vis_canvas, img_data, step=16, scale=20.0)
                cv2.imshow(win_def, vis)

            elif img_type == "shear":
                if vis_canvas is None or vis_canvas.shape[:2] != img_data.shape[:2]:
                    vis_canvas = np.zeros(img_data.shape[:2] + (3,), dtype=np.uint8)
                else:
                    vis_canvas.fill(0)

                vis = put_arrows_on_image(vis_canvas, img_data, step=16, scale=20.0)
                cv2.imshow(win_shear, vis)

            elif img_type == "infer":
                cv2.imshow(win_infer, img_data)

            # Poll hotkeys from the visualization windows
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[Vis] 'q' pressed. Signal stopping...")
                running_event.clear()
                break
            elif key == ord("r"):
                print("[Vis] 'r' pressed in vis window. Request reset...")
                reset_event.set()

    except Exception as e:
        print(f"[Vis] Error: {e}")

    finally:
        cv2.destroyAllWindows()
        print("[Vis] Process exited.")


def main() -> None:
    # On Windows, multiprocess OpenCV visualization can be fragile
    if not IS_WINDOWS:
        try:
            mp.set_start_method("spawn", force=True)
        except RuntimeError:
            pass

    # Remote X5 device configuration (example)
    opt = SensorOptions(
        dev_id=2,
        backend="cpu",
        mode=Mode.STANDARD,
        show_fps=True,
        max_fps=120,
        enable_raw=False,
        enable_deformation=True,
        enable_depth=True,
        enable_shear=True,
        enable_force=True,
        remote_addr="192.168.127.10:50051",
        pc_host="192.168.127.100",
        pc_port=60001,
    )

    sensor = Sensor(opt)

    last_fid = -1
    frame_cnt = 0
    t0 = time.time()

    # =========================
    # Windows: single-process visualization
    # =========================
    if IS_WINDOWS:
        print("[Main] Running in single-process mode on Windows (no mp visualizer).")
        running = True

        try:
            while running:
                # Device status check
                if sensor.getDevStatus() != 0:
                    k = cv2.waitKey(1) & 0xFF
                    if k == ord("q"):
                        running = False
                        break
                    elif k == ord("r"):
                        print("[Main] 'r' pressed in main window.")
                        try:
                            sensor.reset()
                        except Exception as e:
                            print(f"[Main] reset() failed: {e}")
                    time.sleep(0.01)
                    continue

                # Wait for a new frame
                got_new = sensor.wait_for_new(last_fid, timeout_ms=500)
                if opt.backend == "x5":
                    event, _ = sensor.getEvents()

                if not got_new:
                    k = cv2.waitKey(1) & 0xFF
                    if k == ord("q"):
                        running = False
                        break
                    elif k == ord("r"):
                        print("[Main] 'r' pressed in main window.")
                        try:
                            sensor.reset()
                        except Exception as e:
                            print(f"[Main] reset() failed: {e}")
                    continue

                # Read raw/infer for display (if enabled)
                fid, raw = sensor.getRawImg()
                if raw is not None:
                    fid, inf = sensor.getInferImg()
                    if inf is not None and getattr(inf, "img", None) is not None:
                        cv2.imshow("infer_main", inf.img)

                # Visualize deformation
                _, deformation = sensor.getDeformation2D()
                if deformation is not None:
                    canvas_def = np.zeros(deformation.shape[:2] + (3,), dtype=np.uint8)
                    vis_def = put_arrows_on_image(canvas_def, deformation, step=16, scale=20.0)
                    cv2.imshow("deformation", vis_def)

                # Visualize shear
                _, shear = sensor.getShear()
                if shear is not None:
                    canvas_shear = np.zeros(shear.shape[:2] + (3,), dtype=np.uint8)
                    vis_shear = put_arrows_on_image(canvas_shear, shear, step=16, scale=20.0)
                    cv2.imshow("shear", vis_shear)

                # Display depth (scaled for preview)
                fid, depth = sensor.getDepth()
                if depth is not None:
                    depth_img = (depth * 50).clip(0, 255).astype(np.uint8)
                    cv2.imshow("depth", depth_img)

                # Hotkeys from the main window loop
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    print("[Main] 'q' pressed.")
                    running = False
                    break
                elif k == ord("r"):
                    print("[Main] 'r' pressed in main window.")
                    try:
                        sensor.reset()
                    except Exception as e:
                        print(f"[Main] reset() failed: {e}")

                last_fid = fid

                # FPS statistics
                frame_cnt += 1
                now = time.time()
                if now - t0 >= 1.0:
                    fps = frame_cnt / (now - t0)
                    print(f"Display FPS: {fps:.2f}")
                    frame_cnt = 0
                    t0 = now

        except KeyboardInterrupt:
            print("[Main] KeyboardInterrupt.")
            running = False

        finally:
            print("[Main] Exiting (Windows single-process)...")
            try:
                sensor.disconnect()
            except Exception:
                pass
            cv2.destroyAllWindows()
            print("[Main] Done.")
        return

    # =========================
    # Non-Windows: multi-process visualization
    # =========================

    # Keep the queue small to avoid backlog and slow shutdown
    data_queue = mp.Queue(maxsize=3)

    # Avoid implicit join on background feeder thread (can hang on exit)
    data_queue.cancel_join_thread()

    running_event = mp.Event()
    running_event.set()
    reset_event = mp.Event()

    # Start visualization process
    p = mp.Process(
        target=visualizer_process,
        args=(data_queue, running_event, reset_event),
        daemon=True,
    )
    p.start()

    try:
        while running_event.is_set():
            # Handle reset requests coming from the visualizer
            if reset_event.is_set():
                print("[Main] reset requested from visualizer.")
                try:
                    sensor.reset()
                except Exception as e:
                    print(f"[Main] reset() failed: {e}")
                reset_event.clear()

            # Device status check
            if sensor.getDevStatus() != 0:
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    running_event.clear()
                    break
                elif k == ord("r"):
                    print("[Main] 'r' pressed in main window.")
                    try:
                        sensor.reset()
                    except Exception as e:
                        print(f"[Main] reset() failed: {e}")
                time.sleep(0.01)
                continue

            # Wait for a new frame
            got_new = sensor.wait_for_new(last_fid, timeout_ms=500)
            if opt.backend == "x5":
                event, _ = sensor.getEvents()

            if not running_event.is_set():
                break

            if not got_new:
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    running_event.clear()
                    break
                elif k == ord("r"):
                    print("[Main] 'r' pressed in main window.")
                    try:
                        sensor.reset()
                    except Exception as e:
                        print(f"[Main] reset() failed: {e}")
                continue

            # Read raw/infer for optional local display
            fid, raw = sensor.getRawImg()
            if raw is not None:
                fid, inf = sensor.getInferImg()
                if inf is not None and getattr(inf, "img", None) is not None:
                    cv2.imshow("infer_main", inf.img)

            # Non-blocking send to the visualization process
            try:
                _, deformation = sensor.getDeformation2D()
                if deformation is not None:
                    data_queue.put_nowait(("deformation", deformation.copy()))

                _, shear = sensor.getShear()
                if shear is not None:
                    data_queue.put_nowait(("shear", shear.copy()))
            except queue.Full:
                pass
            except Exception:
                pass

            # Show depth locally (preview)
            fid, depth = sensor.getDepth()
            if depth is not None:
                depth_img = (depth * 100).clip(0, 255).astype(np.uint8)
                cv2.imshow("depth", depth_img)

            # Main loop hotkeys (still via OpenCV window events)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                print("[Main] 'q' pressed.")
                running_event.clear()
                break
            elif k == ord("r"):
                print("[Main] 'r' pressed in main window.")
                try:
                    sensor.reset()
                except Exception as e:
                    print(f"[Main] reset() failed: {e}")


            last_fid = fid

            # FPS statistics
            frame_cnt += 1
            now = time.time()
            if now - t0 >= 1.0:
                fps = frame_cnt / (now - t0)
                print(f"Display FPS: {fps:.2f}")
                frame_cnt = 0
                t0 = now

    except KeyboardInterrupt:
        print("[Main] KeyboardInterrupt.")
        running_event.clear()

    finally:
        print("[Main] Exiting (non-Windows mp)...")

        # 1) Signal stop
        running_event.clear()

        # 2) Disconnect sensor
        try:
            sensor.disconnect()
        except Exception:
            pass

        # 3) Close any windows owned by the main process
        cv2.destroyAllWindows()

        # 4) Close queue to avoid deadlocks
        try:
            data_queue.close()
            data_queue.cancel_join_thread()
        except Exception:
            pass

        # 5) Join/terminate visualization process
        p.join(timeout=0.5)
        if p.is_alive():
            print("[Main] Terminating worker process...")
            p.terminate()
            p.join()

        print("[Main] Done.")


if __name__ == "__main__":
    main()
