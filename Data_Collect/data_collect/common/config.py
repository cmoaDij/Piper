"""Configuration and storage paths shared by all stages."""

import json
import math
from pathlib import Path

import yaml

from .paths import ROOT
IMAGE_STREAMS = ("cameras/global_rgb", "cameras/wrist_rgb", "tactile/left_raw", "tactile/right_raw")


def local_path(value):
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def inside_data_collect(value):
    path = local_path(value)
    if not path.is_relative_to(ROOT):
        raise ValueError(f"数据输出必须位于 {ROOT}: {path}")
    return path


def load_config(path=None):
    with local_path(path or "config/collection.yaml").open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["data_root"] = str(inside_data_collect(cfg["data_root"]))
    if cfg["audit"]["fps"] != 20:
        raise ValueError("本方案要求 20 Hz 对齐")
    if cfg["queue_max_items"] <= 0 or cfg["queue_max_mb"] <= 0:
        raise ValueError("队列容量必须为正")
    storage = cfg.setdefault("storage", {})
    for key, default, upper in (("batch_ms", 50, 1000), ("batch_max_items", 128, 2048), ("encoder_threads", 1, 4)):
        value = storage.setdefault(key, default)
        if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= upper:
            raise ValueError(f"storage.{key} 必须为 1–{upper} 的整数")
    if storage.setdefault("video_encoder", "auto") not in ("auto", "h264_nvenc", "libx264"):
        raise ValueError("storage.video_encoder 必须为 auto/h264_nvenc/libx264")
    quality = storage.setdefault("video_quality", 18)
    if not isinstance(quality, int) or isinstance(quality, bool) or not 0 <= quality <= 51:
        raise ValueError("storage.video_quality 必须为 0–51 的整数")
    for camera in cfg["cameras"].values():
        if any(not isinstance(camera[k], int) or isinstance(camera[k], bool) or camera[k] <= 0 for k in ("width", "height", "fps")) or camera["width"] % 2 or camera["height"] % 2:
            raise ValueError("视频要求正整数帧率及正偶数宽高")
    if not 0 < cfg["command_timeout_s"] <= 0.25:
        raise ValueError("command_timeout_s 必须在 (0, 0.25] 秒内")
    if not 0 < cfg["control_hz"] <= cfg["feedback_hz"]:
        raise ValueError("控制频率必须为正且不超过反馈轮询频率")
    for key in ("urdf", "calibration_file"):
        if not local_path(cfg[key]).is_relative_to(ROOT):
            raise ValueError(f"{key} 必须位于 Data_Collect 内")
    if cfg["arm_type"] != "piper_x":
        raise ValueError("当前实机反馈已核对为 Piper-X，请勿使用标准 Piper 模型")
    for key, value in cfg["motion"].items():
        if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"motion.{key} 必须为有限正数")
    if len(cfg["tcp_offset"]) != 6 or not all(math.isfinite(v) for v in cfg["tcp_offset"]):
        raise ValueError("tcp_offset 必须为有限的六维 xyz/rpy")
    return cfg


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8"
    )
    tmp.replace(path)


def fingerprint(path):
    import hashlib

    digest = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
