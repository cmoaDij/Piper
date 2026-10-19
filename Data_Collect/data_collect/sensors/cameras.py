"""Independent native-rate RealSense readers; timestamps are taken under locks."""

import threading
import time

import numpy as np


def realsense_worker(name, options, manager, stop, timeout_ms=500):
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    lock = threading.Lock()
    started = False
    try:
        config = rs.config()
        config.enable_device(options["serial"])
        config.enable_stream(
            rs.stream.color, options["width"], options["height"], rs.format.rgb8, options["fps"]
        )
        pipeline.start(config)
        started = True
        last_id = None
        while not stop.is_set():
            with lock:
                frames = pipeline.wait_for_frames(timeout_ms)
                frame = frames.get_color_frame()
                available = time.monotonic_ns()
                if not frame:
                    continue
                fid = frame.get_frame_number()
                if fid == last_id:
                    continue
                image = np.asanyarray(frame.get_data()).copy()
                device_ts = frame.get_timestamp()  # SDK native milliseconds
                domain = str(frame.get_frame_timestamp_domain())
            last_id = fid
            manager.emit(
                f"cameras/{name}",
                {
                    "data": image,
                    "available_host_ts_ns": np.int64(available),
                    "device_ts": np.float64(device_ts),
                    "frame_id": np.int64(fid),
                },
            )
            # Domain is recorded in configuration once, not as a variable string array.
            options["device_timestamp_domain"] = domain
    except Exception as exc:
        if not stop.is_set():
            manager.fail(f"RealSense {name}: {exc}")
    finally:
        if started:
            pipeline.stop()
