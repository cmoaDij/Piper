#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dmrobotics multi-sensor visualization demo (one worker process per sensor)

What this script does
---------------------
- Starts one Python process per sensor (SENSORS_TO_USE list).
- Each sensor process connects to a remote device (gRPC address) and streams data
  back to the PC host (PC_HOST:pc_port).
- On Windows:
  - Uses a single process per sensor to both acquire data and render OpenCV windows
    (multiprocessing OpenCV visualization is often unstable on Windows).
- On non-Windows (Linux):
  - Each sensor worker spawns an additional visualization subprocess for
    deformation/shear/infer windows.
  - The main worker process continues to handle sensor I/O and sends frames to the
    visualization subprocess via a small multiprocessing queue.

Displayed channels
------------------
- Infer image (BGR/GRAY depending on your SDK binding)
- Deformation (2D flow) rendered as arrows
- Shear (2D curl) rendered as arrows
- Depth (float depth map scaled to 8-bit for preview)

Keyboard controls (per sensor)
------------------------------
- 'q' : quit the current sensor worker (closes its windows)
- 'r' : reset the sensor (requested either from the worker windows or from the
        visualizer subprocess on Linux)

Notes
-----
- The IPC queue is intentionally small to avoid backlog and slow shutdown.
- This demo is intended for interactive visualization. For headless usage, replace
  cv2.waitKey() with terminal key polling (termios + select) and disable imshow().
"""

import time
import multiprocessing as mp
import queue
import cv2
import numpy as np
import os

from dmrobotics import (
    Sensor,
    SensorOptions,
    Mode
)

from dmrobotics.utils import put_arrows_on_image

PC_HOST   = "192.168.127.100"
BASE_PORT = 60000

IS_WINDOWS = (os.name == "nt")

SENSORS_TO_USE = [
    {
        "name": "sensor_0",
        "dev_id": 0,
        "remote_addr": "192.168.127.10:50051",
        "pc_port": BASE_PORT + 0,
    },
    {
        "name": "sensor_1",
        "dev_id": 2,
        "remote_addr": "192.168.127.10:50052",
        "pc_port": BASE_PORT + 1,
    },
]


def visualizer_process(data_queue: mp.Queue, running_event, reset_event, name: str) -> None:
    """
    Dedicated visualization process (non-Windows only).

    Receives tuples from data_queue:
      - ("infer", img)
      - ("deformation", flow)
      - ("shear", curl)

    Hotkeys in the visualization windows:
      - 'q' : stop everything for this sensor
      - 'r' : request sensor reset (sets reset_event)
    """
    vis_canvas = None

    # Window names are prefixed with sensor name to distinguish cameras
    win_def   = f"deformation_{name}"
    win_shear = f"shear_{name}"
    win_infer = f"infer_{name}"

    try:
        while running_event.is_set():
            try:
                # Short timeout so we can react quickly to stop requests
                data = data_queue.get(timeout=0.05)
            except Exception:
                # Even if the queue is empty, still poll hotkeys
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    print(f"[{name}] Visualizer 'q' pressed.")
                    running_event.clear()
                    break
                elif key == ord("r"):
                    print(f"[{name}] Visualizer 'r' pressed.")
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

            # Poll hotkeys from OpenCV windows
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print(f"[{name}] Visualizer 'q' pressed.")
                running_event.clear()
                break
            elif key == ord("r"):
                print(f"[{name}] Visualizer 'r' pressed.")
                reset_event.set()

    except Exception as e:
        print(f"[{name}] Visualizer error: {e}")
    finally:
        cv2.destroyAllWindows()
        print(f"[{name}] Visualizer exited.")


def run_one_sensor(desc: dict) -> None:
    """
    One worker process per sensor.

    - On Windows: this process handles both acquisition and visualization.
    - On non-Windows: this process handles acquisition and spawns a separate
      visualization subprocess for deformation/shear/infer.
    """
    name = desc["name"]

    opt = SensorOptions(
        dev_id=desc["dev_id"],
        backend="cuda",              # remote + CUDA inference
        mode=Mode.STANDARD,
        show_fps=True,
        enable_raw=False,
        enable_deformation=True,
        enable_depth=True,
        enable_shear=True,
        enable_force=False,
        remote_addr=desc["remote_addr"],
        pc_host=PC_HOST,
        pc_port=desc["pc_port"],
    )

    print(f"[{name}] Connecting...")
    sensor = Sensor(opt)

    last_fid = -1
    frame_cnt = 0
    t0 = time.time()

    depth_win_name = f"depth_{name}"
    infer_win_name = f"infer_{name}"
    def_win_name   = f"deformation_{name}"
    shear_win_name = f"shear_{name}"

    # ============================
    # Windows: visualize in the same process
    # ============================
    if IS_WINDOWS:
        print(f"[{name}] Running in single-process mode on Windows (no vis subprocess).")
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
                        print(f"[{name}] 'r' pressed (status!=0).")
                        try:
                            sensor.reset()
                        except Exception as e:
                            print(f"[{name}] reset() failed: {e}")
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
                        print(f"[{name}] 'r' pressed (no new frame).")
                        try:
                            sensor.reset()
                        except Exception as e:
                            print(f"[{name}] reset() failed: {e}")
                    continue

                # Read raw/infer (infer is used for display)
                fid, raw = sensor.getRawImg()
                if raw is not None:
                    fid, inf = sensor.getInferImg()
                    if inf is not None and getattr(inf, "img", None) is not None:
                        cv2.imshow(infer_win_name, inf.img)

                # Deformation visualization
                _, deformation = sensor.getDeformation2D()
                if deformation is not None:
                    canvas_def = np.zeros(deformation.shape[:2] + (3,), dtype=np.uint8)
                    vis_def = put_arrows_on_image(canvas_def, deformation, step=16, scale=20.0)
                    cv2.imshow(def_win_name, vis_def)

                # Shear visualization
                _, shear = sensor.getShear()
                if shear is not None:
                    canvas_shear = np.zeros(shear.shape[:2] + (3,), dtype=np.uint8)
                    vis_shear = put_arrows_on_image(canvas_shear, shear, step=16, scale=20.0)
                    cv2.imshow(shear_win_name, vis_shear)

                # Depth preview
                fid, depth = sensor.getDepth()
                if depth is not None:
                    depth_img = (depth * 50).clip(0, 255).astype(np.uint8)
                    cv2.imshow(depth_win_name, depth_img)

                last_fid = fid

                # Hotkeys
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    print(f"[{name}] 'q' pressed in main loop.")
                    running = False
                    break
                elif k == ord("r"):
                    print(f"[{name}] 'r' pressed in main loop.")
                    try:
                        sensor.reset()
                    except Exception as e:
                        print(f"[{name}] reset() failed: {e}")

                # FPS
                frame_cnt += 1
                now = time.time()
                if now - t0 >= 1.0:
                    fps = frame_cnt / (now - t0)
                    print(f"[{name}] FPS: {fps:.2f}")
                    frame_cnt = 0
                    t0 = now

        except KeyboardInterrupt:
            print(f"[{name}] KeyboardInterrupt.")
            running = False

        finally:
            print(f"[{name}] Cleaning up (Windows)...")
            try:
                sensor.disconnect()
            except Exception:
                pass
            cv2.destroyAllWindows()
            print(f"[{name}] Worker process exited cleanly.")
        return

    # ==================================
    # Non-Windows: spawn a visualization subprocess
    # ==================================

    # Keep the queue small to avoid backlog
    data_queue = mp.Queue(maxsize=3)

    # Do not wait for the queue feeder thread on interpreter shutdown
    data_queue.cancel_join_thread()

    running_event = mp.Event()
    running_event.set()
    reset_event = mp.Event()

    # Start visualization subprocess (daemon)
    p_vis = mp.Process(
        target=visualizer_process,
        args=(data_queue, running_event, reset_event, name),
        name=f"vis-{name}",
        daemon=True
    )
    p_vis.start()

    try:
        while running_event.is_set():
            # Handle reset requests from the visualizer subprocess
            if reset_event.is_set():
                print(f"[{name}] reset requested from visualizer.")
                try:
                    sensor.reset()
                except Exception as e:
                    print(f"[{name}] reset() failed: {e}")
                reset_event.clear()

            # Device status check
            if sensor.getDevStatus() != 0:
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    running_event.clear()
                    break
                elif k == ord("r"):
                    print(f"[{name}] 'r' pressed in depth window (status!=0).")
                    try:
                        sensor.reset()
                    except Exception as e:
                        print(f"[{name}] reset() failed: {e}")
                time.sleep(0.01)
                continue

            # Wait for a new frame
            got_new = sensor.wait_for_new(last_fid, timeout_ms=500)
            if opt.backend == "x5":
                event, _ = sensor.getEvents()

            # Exit check
            if not running_event.is_set():
                break

            if not got_new:
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    running_event.clear()
                    break
                elif k == ord("r"):
                    print(f"[{name}] 'r' pressed in depth window (no new frame).")
                    try:
                        sensor.reset()
                    except Exception as e:
                        print(f"[{name}] reset() failed: {e}")
                continue

            # Read raw/infer and forward infer to visualizer
            fid, raw = sensor.getRawImg()
            if raw is not None:
                fid, inf = sensor.getInferImg()
                if inf is not None and getattr(inf, "img", None) is not None:
                    try:
                        data_queue.put_nowait(("infer", inf.img.copy()))
                    except queue.Full:
                        pass
                    except Exception:
                        pass

            # Forward deformation to visualizer
            _, deformation = sensor.getDeformation2D()
            if deformation is not None:
                try:
                    data_queue.put_nowait(("deformation", deformation.copy()))
                except queue.Full:
                    pass

            # Forward shear to visualizer
            _, shear = sensor.getShear()
            if shear is not None:
                try:
                    data_queue.put_nowait(("shear", shear.copy()))
                except queue.Full:
                    pass

            # Depth preview is displayed in this worker process
            fid, depth = sensor.getDepth()
            last_fid = fid
            if depth is not None:
                depth_img = (depth * 50).clip(0, 255).astype(np.uint8)
                cv2.imshow(depth_win_name, depth_img)

            # Hotkeys (captured from the depth window)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                print(f"[{name}] 'q' pressed in depth window.")
                running_event.clear()
                break
            elif k == ord("r"):
                print(f"[{name}] 'r' pressed in depth window.")
                try:
                    sensor.reset()
                except Exception as e:
                    print(f"[{name}] reset() failed: {e}")

            # FPS
            frame_cnt += 1
            now = time.time()
            if now - t0 >= 1.0:
                fps = frame_cnt / (now - t0)
                print(f"[{name}] FPS: {fps:.2f}")
                frame_cnt = 0
                t0 = now

    except KeyboardInterrupt:
        print(f"[{name}] KeyboardInterrupt.")
        running_event.clear()

    finally:
        print(f"[{name}] Cleaning up (non-Windows)...")
        running_event.clear()

        # Disconnect sensor
        try:
            sensor.disconnect()
        except Exception:
            pass

        # Close windows owned by this worker process
        cv2.destroyAllWindows()

        # Close queue resources
        try:
            data_queue.close()
            data_queue.cancel_join_thread()
        except Exception:
            pass

        # Ensure visualizer subprocess exits
        p_vis.join(timeout=1.0)
        if p_vis.is_alive():
            print(f"[{name}] Terminating visualizer...")
            p_vis.terminate()
            p_vis.join()

        print(f"[{name}] Worker process exited cleanly.")


def main():
    # 'spawn' is safer for multiprocessing + CUDA; Windows also uses spawn by default
    try:
        mp.set_start_method("spawn", force=True)
    except RuntimeError:
        pass

    procs = []
    for desc in SENSORS_TO_USE:
        p = mp.Process(
            target=run_one_sensor,
            args=(desc,),
            name=f"sensor-{desc['name']}",
        )
        p.start()
        procs.append(p)

    print("[main] All sensor workers started. Press 'q' in the corresponding window to close a sensor.")
    print("[main] The main process exits only after ALL sensor workers have exited.")

    try:
        for p in procs:
            p.join()
    except KeyboardInterrupt:
        print("\n[main] KeyboardInterrupt, terminating all workers...")
        for p in procs:
            if p.is_alive():
                p.terminate()
    finally:
        print("[main] All workers finished. Exiting.")


if __name__ == "__main__":
    main()
