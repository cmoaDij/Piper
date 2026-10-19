"""Preflight the approved index cache, then export through official LeRobot v3 APIs."""

import inspect
import json
import os
from pathlib import Path

import h5py
import numpy as np

from .config import IMAGE_STREAMS, ROOT, atomic_json, fingerprint, inside_data_collect
from .images import EpisodeImages, image_shape, validate_images

FEATURE_IMAGES = dict(
    zip(
        IMAGE_STREAMS,
        (
            "observation.images.camera_global",
            "observation.images.camera_wrist",
            "observation.images.tactile_left",
            "observation.images.tactile_right",
        ),
    )
)
JOINT_NAMES = [f"q{i}" for i in range(1, 7)] + ["gripper_m"]
EEF_NAMES = ["x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad", "gripper_m"]


def preflight(manifest_path, allow_simulated=False):
    manifest_path = Path(manifest_path).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != "piper_manifest_v1" or manifest.get("fps") != 20:
        raise ValueError("白名单格式错误，或不是 20Hz")
    if not manifest.get("episodes"):
        raise ValueError("白名单为空；请先完成自动审计和人工通过")
    result, seen = [], set()
    shapes = None
    for entry in manifest["episodes"]:
        path = Path(entry["path"])
        sync_path = Path(entry["sync_path"])
        if not path.is_absolute():
            path = manifest_path.parent / path
        if not sync_path.is_absolute():
            sync_path = manifest_path.parent / sync_path
        path, sync_path = path.resolve(), sync_path.resolve()
        if path in seen or entry["split"] not in ("train", "val"):
            raise ValueError("重复 Episode 或未知 train/val 标签")
        seen.add(path)
        if (
            fingerprint(path) != entry["source_sha256"]
            or fingerprint(sync_path) != entry["sync_sha256"]
        ):
            raise ValueError(f"文件已变更，请重新审计/人工确认: {path.name}")
        sync = json.loads(sync_path.read_text(encoding="utf-8"))
        if (
            sync.get("schema_version") != "piper_sync_v1"
            or sync.get("automatic_status") != "PASS"
            or sync.get("fps") != 20
            or sync["source_sha256"] != entry["source_sha256"]
            or Path(sync["source_path"]).resolve() != path
        ):
            raise ValueError("索引缓存与白名单不符")
        grid = np.asarray(sync["grid_host_ts_ns"], dtype=np.int64)
        if len(grid) == 0 or np.any(np.diff(grid) != 50_000_000):
            raise ValueError("索引缓存不是完整的 20Hz 网格")
        with h5py.File(path, "r") as f:
            if not f.attrs.get("complete") or f.attrs.get("collection_error"):
                raise ValueError("白名单包含未完整采集的文件")
            if f.attrs.get("simulated", False) and not allow_simulated:
                raise ValueError("包含合成数据；仅演练时显式指定 --allow-simulated")
            expected = np.arange(
                int(f.attrs["start_host_ts_ns"]),
                int(f.attrs["end_host_ts_ns"]),
                50_000_000,
                dtype=np.int64,
            )
            if not np.array_equal(grid, expected):
                raise ValueError("缓存切掉了 Episode 开头或结尾")
            validate_images(f, decode=True)
            current_shapes = {s: image_shape(f[s]) for s in IMAGE_STREAMS}
            if shapes is not None and current_shapes != shapes:
                raise ValueError("Episode 间图像尺寸不一致")
            shapes = current_shapes
            for group in (*IMAGE_STREAMS, "robot_feedback", "robot_command"):
                idx = np.asarray(sync["indices"][group])
                ts_key = "cmd_call_begin_ns" if group == "robot_command" else "available_host_ts_ns"
                ts = f[group + "/" + ts_key][:]
                if (
                    idx.dtype.kind not in "iu"
                    or idx.shape != grid.shape
                    or (idx < 0).any()
                    or (idx >= len(ts)).any()
                ):
                    raise ValueError(f"{group} 缓存索引越界或长度不符")
                if group != "robot_command" and (ts[idx] > grid).any():
                    raise ValueError("观测缓存引用了未来帧")
        result.append({**entry, "path": str(path), "sync_path": str(sync_path), "sync": sync})
    # Both partitions share one continuous episode index, with contiguous split ranges.
    result.sort(key=lambda entry: entry["split"] == "val")
    return manifest, result, shapes


def feature_schema(shapes):
    features = {
        key: {"dtype": "float32", "shape": (7,), "names": names}
        for key, names in (
            ("observation.state", JOINT_NAMES),
            ("action", JOINT_NAMES),
            ("observation.eef_pose", EEF_NAMES),
            ("action.eef_pose", EEF_NAMES),
        )
    }
    for stream, key in FEATURE_IMAGES.items():
        # Tac-X is stored as lossless PNG-backed image features, embedded in Parquet by LeRobot.
        features[key] = {
            "dtype": "video" if stream.startswith("cameras/") else "image",
            "shape": tuple(shapes[stream]),
            "names": ["height", "width", "channels"],
        }
    return features


def aligned_frames(file, sync):
    with EpisodeImages(file) as images:
        yield from _aligned_frames(file, sync, images)


def _aligned_frames(file, sync, images):
    for k in range(len(sync["grid_host_ts_ns"])):
        i = sync["indices"]["robot_feedback"][k]
        j = sync["indices"]["robot_command"][k]
        actual_grip = file["robot_feedback/gripper_pos"][i]
        target_grip = file["robot_command/target_gripper_pos"][j]
        frame = {
            "observation.state": np.r_[file["robot_feedback/joint_pos"][i], actual_grip].astype(
                np.float32
            ),
            "observation.eef_pose": np.r_[file["robot_feedback/tcp_pose"][i], actual_grip].astype(
                np.float32
            ),
            "action": np.r_[file["robot_command/target_joint_pos"][j], target_grip].astype(
                np.float32
            ),
            "action.eef_pose": np.r_[file["robot_command/target_tcp_pose"][j], target_grip].astype(
                np.float32
            ),
            "task": str(file.attrs["task_instruction"]),
        }
        for stream, key in FEATURE_IMAGES.items():
            frame[key] = images.read(stream, sync["indices"][stream][k])
        yield frame


def convert(manifest_path, output, repo_id, allow_simulated=False, dataset_class=None):
    _, entries, shapes = preflight(manifest_path, allow_simulated)
    output = inside_data_collect(output)
    staging = output.with_name(output.name + ".partial")
    if output.exists() or staging.exists():
        raise FileExistsError(f"输出或中断目录已存在：{output} / {staging}；请使用新目录")
    output.parent.mkdir(parents=True, exist_ok=True)
    if dataset_class is None:
        os.environ.setdefault("HF_HOME", str(ROOT / "data" / "cache" / "huggingface"))
        from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset

        if CODEBASE_VERSION != "v3.0":
            raise RuntimeError("需要支持 v3.0 的 LeRobot，建议安装 lerobot==0.4.4")
        dataset_class = LeRobotDataset
    params = inspect.signature(dataset_class.create).parameters
    kwargs = {
        "repo_id": repo_id,
        "fps": 20,
        "features": feature_schema(shapes),
        "root": staging,
        "robot_type": "piper",
        "use_videos": True,
        "image_writer_threads": 0,
    }
    if "vcodec" in params:
        kwargs["vcodec"] = "h264"
    elif "rgb_encoder" in params:
        from lerobot.configs.video import RGBEncoderConfig

        kwargs["rgb_encoder"] = RGBEncoderConfig(vcodec="h264")
    else:
        raise RuntimeError("LeRobot API 无法指定 H.264 编码器")
    dataset = None
    mapping = []
    try:
        dataset = dataset_class.create(**kwargs)
        for index, entry in enumerate(entries):
            with h5py.File(entry["path"], "r") as file:
                for frame in aligned_frames(file, entry["sync"]):
                    dataset.add_frame(frame)
                dataset.save_episode(parallel_encoding=False)
            mapping.append(
                {
                    "episode_index": index,
                    "source_path": entry["path"],
                    "source_sha256": entry["source_sha256"],
                    "split": entry["split"],
                    "frames": len(entry["sync"]["grid_host_ts_ns"]),
                }
            )
        dataset.finalize()
        info_path = staging / "meta" / "info.json"
        info = json.loads(info_path.read_text(encoding="utf-8"))
        if info.get("codebase_version") != "v3.0" or info.get("total_episodes") != len(entries):
            raise RuntimeError("LeRobot 封盘结果不符合 v3.0 或 Episode 数目错误")
        if info.get("total_frames") != sum(e["frames"] for e in mapping):
            raise RuntimeError("LeRobot 帧数错误")
        if not (staging / "meta" / "stats.json").is_file():
            raise RuntimeError("LeRobot 未输出归一化统计")
        ntrain = sum(e["split"] == "train" for e in entries)
        info["splits"] = {}
        if ntrain:
            info["splits"]["train"] = f"0:{ntrain}"
        if ntrain < len(entries):
            info["splits"]["val"] = f"{ntrain}:{len(entries)}"
        atomic_json(info_path, info)
        atomic_json(staging / "meta" / "source_episodes.json", mapping)
        atomic_json(
            staging / "meta" / "split_episodes.json",
            {
                split: [m["episode_index"] for m in mapping if m["split"] == split]
                for split in ("train", "val")
            },
        )
        staging.rename(output)
        return output
    except Exception as exc:
        if staging.exists():
            atomic_json(
                staging / "conversion_error.json",
                {"error": str(exc), "completed_episodes": len(mapping)},
            )
        raise
    finally:
        if dataset is not None and hasattr(dataset, "stop_image_writer"):
            dataset.stop_image_writer()
