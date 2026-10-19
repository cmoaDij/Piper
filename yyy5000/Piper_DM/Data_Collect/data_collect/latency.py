"""Measure relative image-stream delay and apply availability-safe alignment.

Offsets are ADDED to SDK-return timestamps. Prefer the latest stream as the
reference: delaying early streams aligns them without inventing early arrival.
An aligned timestamp is not a physical exposure timestamp.
"""

import argparse
import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import h5py
import numpy as np

from .config import IMAGE_STREAMS, atomic_json, fingerprint, inside_data_collect, load_config, local_path

STREAM_NAMES = tuple(s.split("/")[1] for s in IMAGE_STREAMS)
TIMESTAMP_POLICY = "available_and_aligned_not_after_grid_v1"


def image_offsets(calibration):
    values = calibration.get("relative_offsets_ns", {})
    if not isinstance(values, dict) or set(values) - set(STREAM_NAMES):
        raise ValueError("relative_offsets_ns 包含未知流或格式错误")
    result = {}
    for name in STREAM_NAMES:
        value = values.get(name, 0)
        if isinstance(value, bool) or not isinstance(value, int) or abs(value) > 5_000_000_000:
            raise ValueError(f"{name} 偏移必须为整数纳秒，范围 ±5 秒")
        result[name] = value
    return result


def aligned_timestamps(raw, stream, calibration):
    raw = np.asarray(raw)
    if raw.dtype.kind not in "iu" or raw.size == 0 or np.any(raw <= 0):
        raise ValueError("原始时间戳必须为非空正整数")
    offset = image_offsets(calibration)[stream.split("/")[-1]]
    maximum = np.iinfo(np.int64).max
    if int(raw.max()) > maximum - max(offset, 0) or int(raw.min()) + offset <= 0:
        raise ValueError("补偿时间戳越界")
    result = raw.astype(np.int64) + offset
    if (np.diff(result) < 0).any():
        raise ValueError("补偿时间戳必须单调")
    return result


def image_indices(raw, grid, valid, stream, calibration):
    from .alignment import causal_indices

    corrected = aligned_timestamps(raw, stream, calibration)
    # Negative offsets must never make a frame available before SDK return.
    indices, _ = causal_indices(np.maximum(raw, corrected), grid, valid)
    return indices, np.asarray(grid) - raw[indices], corrected[indices]


def stored_calibration(file):
    result = json.loads(file.attrs.get("calibration", "{}"))
    image_offsets(result)
    return result


def check_devices(profile, recorded):
    for key in ("camera_serials", "tactile_ids"):
        if key not in profile or profile[key] != recorded.get(key):
            raise ValueError(f"延迟标定 {key} 与原始文件不匹配")


def _series(group):
    ts = group["available_host_ts_ns"][:]
    fid = group["frame_id"][:]
    if ts.dtype.kind not in "iu" or len(ts) < 3 or len(fid) != len(ts):
        raise ValueError(f"{group.name} 缺少足够有效帧")
    if (ts <= 0).any() or (np.diff(ts) < 0).any():
        raise ValueError(f"{group.name} 时间戳非法")
    _, first = np.unique(fid, return_index=True)
    first.sort()
    return ts[first].astype(np.int64), fid[first], first


def _features(group):
    ts, _, rows = _series(group)
    data = group["data"]
    if data.ndim != 4 or data.shape[-1] != 3 or len(data) != len(group["frame_id"]):
        raise ValueError(f"{group.name} 图像格式错误")
    previous, features = None, []
    for row in rows:
        frame = data[int(row)][::8, ::8].astype(np.float32)
        features.append(0.0 if previous is None else float(np.abs(frame - previous).mean()))
        previous = frame
    return ts, np.asarray(features)


def _scan_delay(left, right, max_ms, min_score):
    """Compare x(t) with y(t+lag); a late right stream has positive lag."""
    from scipy.signal import find_peaks

    tx, x = left
    ty, y = right
    origin = min(int(tx[0]), int(ty[0]))
    tx, ty = (tx - origin) / 1e9, (ty - origin) / 1e9
    limit, step = max_ms / 1000, .005
    start, end = max(tx[0], ty[0]) + limit, min(tx[-1], ty[-1]) - limit
    if end - start < 5:
        raise ValueError("共同有效片段不足 5 秒，请录制至少 10–20 秒")
    for signal in (x, y):
        if np.std(signal) < .01 or len(find_peaks(signal, height=np.mean(signal), distance=4)[0]) < 5:
            raise ValueError("有效变化不足，请在四路中制造至少 5 次不等间隔共同事件")
    lags = np.arange(-limit, limit + step / 2, step)
    grid = np.arange(start, end, step)
    values = np.interp(grid, tx, x)
    curves = []
    for mask in (np.ones(len(grid), dtype=bool), grid < (start+end)/2, grid >= (start+end)/2):
        a = values[mask] - values[mask].mean()
        scores = []
        for lag in lags:
            b = np.interp(grid[mask] + lag, ty, y)
            b -= b.mean()
            norm = np.linalg.norm(a) * np.linalg.norm(b)
            scores.append(float(a @ b / norm) if norm > 1e-12 else -1.)
        curves.append(np.asarray(scores))
    best = int(np.argmax(curves[0]))
    score = float(curves[0][best])
    if best in (0, len(lags)-1) or score < min_score:
        raise ValueError(f"共同事件相关性不足或峰值触及搜索边界: score={score:.3f}")
    alternatives = np.abs(lags - lags[best]) >= .050
    margin = score - float(curves[0][alternatives].max()) if alternatives.any() else 1.
    if margin < .05:
        raise ValueError("多个延迟候选接近，可能是周期性动作；请改用不等间隔事件或人工事件标注")
    spread = max(abs(float(lags[np.argmax(c)] - lags[best])) for c in curves[1:])
    if spread > .040 or min(float(c.max()) for c in curves[1:]) < min_score:
        raise ValueError("前后半段估计不一致，延迟不稳定或动作并非共同事件")
    return int(round(lags[best] * 1e9)), {
        "correlation": score, "peak_margin": margin, "half_spread_ms": spread*1000,
        "search_step_ms": step*1000,
    }


def measure(path, events=None, reference="latest", max_ms=500., min_score=.65):
    path = local_path(path)
    with h5py.File(path, "r") as file:
        if not file.attrs.get("complete") or file.attrs.get("collection_error") or file.attrs.get("dropped_samples", 0):
            raise ValueError("标定需要完整、无采集错误、无丢样的原始文件")
        recorded = stored_calibration(file)
        quality = {}
        if events:
            # Each event must describe the SAME physical instant in all four streams.
            rows = {}
            with local_path(events).open(encoding="utf-8-sig", newline="") as source:
                for row in csv.DictReader(source):
                    name = row["stream"].split("/")[-1]
                    if name not in STREAM_NAMES or not row["event_id"].strip():
                        raise ValueError("事件 CSV 的 stream 或 event_id 非法")
                    event = rows.setdefault(row["event_id"], {})
                    if name in event:
                        raise ValueError("同一个事件和传感器被重复标注")
                    ts, ids, _ = _series(file[IMAGE_STREAMS[STREAM_NAMES.index(name)]])
                    match = np.flatnonzero(ids == int(row["frame_id"]))
                    if len(match) != 1:
                        raise ValueError(f"找不到唯一 frame_id: {name}/{row['frame_id']}")
                    event[name] = int(ts[match[0]])
            if len(rows) < 5 or any(set(row) != set(STREAM_NAMES) for row in rows.values()):
                raise ValueError("至少标注 5 个事件，每个事件须包含四路帧")
            marks = np.asarray([[row[n] for n in STREAM_NAMES] for row in rows.values()], dtype=np.int64)
            if any(np.any(np.diff(marks[:, i]) <= 0) for i in range(4)):
                raise ValueError("事件须按发生顺序标注，且不能重复使用同一帧")
            delays = np.median(marks - marks[:, :1], axis=0)
            ref = int(np.argmax(delays)) if reference == "latest" else STREAM_NAMES.index(reference)
            offsets = {}
            for i, name in enumerate(STREAM_NAMES):
                differences = marks[:, ref] - marks[:, i]
                offset = int(round(float(np.median(differences))))
                residual = float(np.percentile(np.abs(differences-offset), 95) / 1e6)
                if residual > 40 or abs(offset) > max_ms*1e6:
                    raise ValueError(f"{name} 偏移或事件间抖动超限: P95残差={residual:.1f} ms")
                offsets[name] = offset
                quality[name] = {"events": len(rows), "residual_p95_ms": residual}
            method = "shared_event_frame_ids"
        else:
            features = {name: _features(file[stream]) for name, stream in zip(STREAM_NAMES, IMAGE_STREAMS)}
            delays = [0]
            quality[STREAM_NAMES[0]] = {"correlation": 1.}
            for name in STREAM_NAMES[1:]:
                lag, quality[name] = _scan_delay(features[STREAM_NAMES[0]], features[name], max_ms, min_score)
                delays.append(lag)
            ref = int(np.argmax(delays)) if reference == "latest" else STREAM_NAMES.index(reference)
            offsets = {name: int(delays[ref] - lag) for name, lag in zip(STREAM_NAMES, delays)}
            method = "shared_change_cross_correlation"
        result = {
            "schema_version": "piper_latency_v1", "measured_at": datetime.now(timezone.utc).isoformat(),
            "method": method, "source_path": str(path), "source_sha256": fingerprint(path),
            "reference_stream": STREAM_NAMES[ref], "relative_offsets_ns": offsets,
            "sign_convention": "aligned_host_ts_ns = available_host_ts_ns + relative_offsets_ns",
            "quality": quality, "timestamp_policy": TIMESTAMP_POLICY,
            "camera_serials": recorded.get("camera_serials"), "tactile_ids": recorded.get("tactile_ids"),
            "absolute_latency_measured": False,
        }
        if events:
            result.update(events_path=str(local_path(events)), events_sha256=fingerprint(local_path(events)))
        image_offsets(result)
        return result


def record(cfg, output, seconds):
    """Images only: no ROS, CAN, arm connection, enabling or motion commands."""
    from .capture import CaptureRuntime, CaptureWriter
    from .storage import EpisodeManager

    if not 10 <= seconds <= 120:
        raise ValueError("标定录制时长须在 10–120 秒")
    output = inside_data_collect(output)
    if output.suffix != ".h5" or output.exists() or output.with_suffix(".h5.partial").exists():
        raise ValueError("输出须为未占用的 .h5 路径")
    output.parent.mkdir(parents=True, exist_ok=True)
    reservation = output.with_suffix(".reserve")
    with reservation.open("x"):
        pass
    cfg["calibration"] = json.loads(local_path(cfg["calibration_file"]).read_text(encoding="utf-8"))
    image_offsets(cfg["calibration"])
    manager = EpisodeManager(cfg)
    runtime = CaptureRuntime(cfg, manager)
    started = False
    try:
        runtime.start()
        started = True
        deadline = time.monotonic() + 15
        while not runtime.ready:
            if manager.failure:
                raise RuntimeError(manager.failure)
            if time.monotonic() > deadline:
                raise RuntimeError("四路图像/触觉基线等待超时")
            time.sleep(.02)
        with manager.lock:
            for stream in IMAGE_STREAMS:
                age = time.monotonic_ns() - int(manager.latest[stream]["available_host_ts_ns"])
                limit = cfg["audit"]["rgb_red_ms" if stream.startswith("cameras/") else "tactile_red_ms"] * 1e6
                if not 0 <= age <= limit:
                    raise RuntimeError(f"标定开始时 {stream} 已过期")
            metadata = {
                "purpose": "latency_calibration", "episode_id": output.stem,
                "task_instruction": "至少5次不等间隔共同事件", "operator": cfg["operator"],
                "start_host_ts_ns": time.monotonic_ns(), "calibration": cfg["calibration"],
                "config": cfg, "simulated": False,
            }
            manager.current = CaptureWriter(runtime, output, metadata, cfg["queue_max_items"], int(cfg["queue_max_mb"]*1024**2))
        print(f"开始四路图像标定录制，{seconds:g} 秒；机械臂未连接。请制造至少 5 次不等间隔共同事件。", flush=True)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if manager.failure:
                raise RuntimeError(manager.failure)
            time.sleep(.02)
        manager.stop()
    except BaseException as exc:
        manager.fail(f"标定录制中止: {type(exc).__name__}: {exc}")
        raise
    finally:
        try:
            if started:
                runtime.close()
        finally:
            reservation.unlink(missing_ok=True)
    return output


def main():
    parser = argparse.ArgumentParser(description="四路图像相对延迟测量；偏移用于离线因果时间补偿")
    parser.add_argument("--config", default="config/collection.yaml")
    commands = parser.add_subparsers(dest="command", required=True)
    recording = commands.add_parser("record", help="仅录制图像，不连接机械臂")
    recording.add_argument("--seconds", type=float, default=20)
    recording.add_argument("--output", default=None)
    measurement = commands.add_parser("measure", help="共同变化自动估计，或使用共同事件 CSV")
    measurement.add_argument("--input", required=True)
    measurement.add_argument("--events", help="CSV 列为 event_id,stream,frame_id；至少5个共同事件")
    measurement.add_argument("--reference", choices=("latest", *STREAM_NAMES), default="latest")
    measurement.add_argument("--max-delay-ms", type=float, default=500)
    measurement.add_argument("--min-correlation", type=float, default=.65)
    measurement.add_argument("--report", default="data/calibration/latency_report.json")
    measurement.add_argument("--write", action="store_true", help="测量通过后写入配置指定的标定文件")
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        if args.command == "record":
            name = args.output or f"data/calibration/latency_{datetime.now():%Y%m%d_%H%M%S}.h5"
            print(record(cfg, name, args.seconds))
        else:
            if not np.isfinite(args.max_delay_ms) or not 50 <= args.max_delay_ms <= 5000 or not .1 <= args.min_correlation <= 1:
                raise ValueError("搜索范围须为 50–5000 ms，相关性门限须为 0.1–1")
            result = measure(args.input, args.events, args.reference, args.max_delay_ms, args.min_correlation)
            if args.write:
                target = local_path(cfg["calibration_file"])
                profile = json.loads(target.read_text(encoding="utf-8"))
                check_devices(profile, result)
                profile["relative_offsets_ns"] = result["relative_offsets_ns"]
                profile["latency_calibration"] = result
                atomic_json(target, profile)
            atomic_json(inside_data_collect(args.report), result)
            print(json.dumps(result, ensure_ascii=False, indent=2))
    except (ValueError, OSError, KeyError, RuntimeError) as exc:
        parser.exit(1, f"延迟标定失败，未产生有效测量：{exc}\n")


if __name__ == "__main__":
    main()
