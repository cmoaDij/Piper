"""Tac-X raw-only adapter for the repository's dmrobotics SDK."""

import threading
import time

import numpy as np


def _raw_image(value, color_order):
    image = np.asarray(value)
    if image.dtype != np.uint8:
        raise ValueError("Tac-X raw 必须为 uint8")
    # Preserve SDK grayscale directly; three-channel expansion belongs to export.
    if image.ndim == 2:
        return image.copy()
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Tac-X raw 必须为 HxW 或 HxWx3")
    return image[..., ::-1].copy() if color_order.lower() == "bgr" else image.copy()


def tactile_worker(name, options, manager, stop):
    from dmrobotics import Mode, Sensor, SensorOptions

    sensor = None
    lock = threading.Lock()
    try:
        opt = SensorOptions(
            **{k: v for k, v in options.items() if k not in ("mode", "color_order")},
            mode=Mode.HIGH if options.get("mode") == "high" else Mode.STANDARD,
            enable_raw=True,
            enable_deformation=False,
            enable_shear=False,
            enable_depth=False,
            enable_force=False,
        )
        sensor = Sensor(opt)
        last_id = -1
        last_new = time.monotonic()
        stream = f"tactile/{name}"
        while not stop.is_set():
            if sensor.getDevStatus() != 0 or not sensor.wait_for_new(last_id, timeout_ms=250):
                # The SDK initializes its no-contact baseline asynchronously.
                # Allow startup to finish; retain the 2 s runtime fault limit.
                timeout_s = 10 if last_id < 0 else 2
                if time.monotonic() - last_new > timeout_s:
                    raise RuntimeError(f"超过 {timeout_s} 秒无新原始帧或设备未就绪")
                stop.wait(0.005)
                continue
            with lock:
                fid, raw = sensor.getRawImg()
                available = time.monotonic_ns()
                if raw is None or getattr(raw, "img", None) is None or fid == last_id:
                    continue
                image = _raw_image(raw.img, options.get("color_order", "bgr"))
                if stream not in manager.baselines:
                    base = sensor.getBaseFrame()
                    # SDK deployments expose either DmTacImage or (fid, DmTacImage).
                    if isinstance(base, tuple):
                        base = base[1]
                    if base is None or getattr(base, "img", None) is None:
                        raise RuntimeError("无有效无接触基线，请先完成 SDK 标定")
                    baseline = _raw_image(base.img, options.get("color_order", "bgr"))
                    if baseline.shape != image.shape:
                        raise ValueError("Tac-X 无接触基线与 raw 帧尺寸不一致")
                    with manager.lock:
                        manager.baselines[stream] = baseline
            last_id = fid
            last_new = time.monotonic()
            manager.emit(
                stream,
                {
                    "data": image,
                    "available_host_ts_ns": np.int64(available),
                    "frame_id": np.int64(fid),
                },
            )
    except Exception as exc:
        if not stop.is_set():
            manager.fail(f"Tac-X {name}: {exc}")
    finally:
        if sensor:
            sensor.disconnect()
