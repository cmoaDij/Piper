"""UMI-style GoPro HDMI/UVC capture, independent of control and disk writing."""

import time

import numpy as np


def gopro_worker(name, options, manager, stop, timeout_ms=500):
    import cv2

    cap = cv2.VideoCapture(options["device"], cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"无法打开 GoPro 采集卡: {options['device']}")
        # Same settings/dequeue pattern as UMI CameraReader and its QR tool.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        mode_properties = ((cv2.CAP_PROP_FRAME_WIDTH, "width"),
                           (cv2.CAP_PROP_FRAME_HEIGHT, "height"), (cv2.CAP_PROP_FPS, "fps"))
        for prop, key in mode_properties:
            cap.set(prop, options[key])
        # V4L2 negotiates width/height together. A width-only intermediate
        # request can be clamped until the requested height is also set.
        for prop, key in mode_properties:
            reported = cap.get(prop)
            # V4L2 may report 59.94 for a nominal 60 Hz HDMI stream. Pairing
            # uses real timestamps, so fractional cadence is not rounded away.
            accepted = (np.isfinite(reported) and round(reported) == options[key]
                        if key == "fps" else reported == options[key])
            if not accepted:
                raise RuntimeError(f"GoPro 采集卡未接受 {key}={options[key]}，请检查视频输出模式")
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        frame_id = 0
        while not stop.is_set():
            ok, bgr = cap.read()
            received = time.monotonic_ns()
            if not ok:
                raise RuntimeError("GoPro 读取失败，请检查 HDMI 和采集卡连接")
            if bgr.shape != (options["height"], options["width"], 3):
                raise RuntimeError(f"GoPro 帧尺寸与配置不符: {bgr.shape}")
            record = {"data": cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
                      "available_host_ts_ns": np.int64(received), "frame_id": np.int64(frame_id)}
            # No guessed offset: uncalibrated capture can warm up, but the
            # collector refuses to start an episode until validation succeeds.
            if "receive_latency_ns" in options:
                record["capture_host_ts_ns"] = np.int64(received - options["receive_latency_ns"])
            manager.emit(f"cameras/{name}", record)
            frame_id += 1
    except Exception as exc:
        manager.fail(f"{name}: {exc}")
    finally:
        cap.release()
