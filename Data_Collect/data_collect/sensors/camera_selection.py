"""Discover capture devices without opening video streams or touching CAN."""

import copy
import json
import math
from pathlib import Path

from data_collect.common.config import fingerprint, local_path


def connected_realsense():
    try:
        import pyrealsense2 as rs
    except ImportError:
        return set()
    return {device.get_info(rs.camera_info.serial_number)
            for device in rs.context().query_devices()}


def connected_gopro(directory="/dev/v4l/by-id"):
    """UMI uses stable V4L2 index0 paths; never auto-select a laptop webcam.

    HDMI cards identify themselves, not the HDMI source. Recognized capture
    cards here are assumed to have the GoPro attached; other cards need an
    explicit by-id device in configuration.
    """
    markers = ("gopro", "elgato", "hd60", "hdmi", "usb_video", "ugreen_35287")
    return [str(path) for path in sorted(Path(directory).glob("*video-index0"))
            if path.exists() and any(marker in path.name.lower() for marker in markers)]


def resolve_cameras(cfg, *, realsense_serials=None, gopro_devices=None):
    """Freeze one profile for the session; never switch roles during an episode."""
    selection = cfg.get("camera_selection", {})
    mode = selection.get("mode", "realsense")
    if mode == "auto":
        serials = connected_realsense() if realsense_serials is None else realsense_serials
        mode = ("realsense" if all(str(c["serial"]) in serials for c in cfg["cameras"].values())
                else "gopro")
    if mode == "realsense":
        for opts in cfg["cameras"].values():
            opts["backend"] = "realsense"
    else:
        profile = selection.get("gopro", {})
        cameras = copy.deepcopy(profile.get("cameras", {}))
        if not cameras:
            raise ValueError("未找到配置中的 RealSense，且未配置 GoPro 采集方案")
        used = set()
        for name, opts in cameras.items():
            device = opts.get("device", "auto")
            if device == "auto":
                candidates = connected_gopro() if gopro_devices is None else gopro_devices
                if len(cameras) != 1 or len(candidates) != 1:
                    prefix = "配置中的 RealSense 设备未匹配；" if selection.get("mode") == "auto" else ""
                    raise ValueError(prefix + f"GoPro 自动识别找到 {len(candidates)} 路；请检查设备连接，并用 /dev/v4l/by-id 明确各相机位置（不会使用笔记本摄像头）")
                device = candidates[0]
            path = Path(device)
            if not path.is_absolute():
                raise ValueError("GoPro device 必须为绝对 V4L2 设备路径")
            if not path.exists() and (gopro_devices is None or device not in gopro_devices):
                raise ValueError(f"GoPro 采集设备未连接: {device}")
            if str(path.resolve()) in used:
                raise ValueError("两个相机位置不能使用同一个采集设备")
            used.add(str(path.resolve()))
            opts.update(backend="gopro", device=str(path), serial=str(path),
                        device_timestamp_domain="host_monotonic_receive")
        cfg["cameras"] = cameras
        for key in ("calibration_id", "calibration_file"):
            if key in profile:
                cfg[key] = profile[key]
    cfg["camera_mode"] = mode
    print("相机方案: " + mode + "；" + ", ".join(f"{name}={c['serial']}" for name, c in cfg["cameras"].items()), flush=True)
    return mode


def load_gopro_latency(options):
    """Read the measurement emitted by UMI's QR receive-latency tool."""
    path = local_path(options["receive_latency_file"])
    profile = json.loads(path.read_text(encoding="utf-8"))
    if (profile["camera"] != options["device"]
            or profile["resolution"] != [options["width"], options["height"]]
            or profile["capture_fps"] != options["fps"]):
        raise ValueError("GoPro 延迟标定与当前设备路径、分辨率或帧率不符，请按当前采集参数重新标定")
    value = profile["nominal_latency_ms"]
    samples = profile.get("sample_count", 0)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0
            or isinstance(samples, bool) or not isinstance(samples, int) or samples < 5):
        raise ValueError("GoPro 缺少有效的实测接收延迟；不能猜测图像与动作的时间偏移")
    options.update(receive_latency_ns=int(round(value * 1e6)),
                   receive_latency_sha256=fingerprint(path), receive_latency_profile=profile)

