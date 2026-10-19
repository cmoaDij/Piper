"""Whole-episode quality gates, persistent index maps and human-approved manifests."""

import csv
import json
from pathlib import Path

import h5py
import numpy as np
from av.error import FFmpegError

from .alignment import action_indices, causal_indices
from .config import IMAGE_STREAMS, atomic_json, fingerprint
from .images import validate_images


def checked_series(group, name, shape=None, finite=True):
    ds = group[name]
    arr = ds[:]
    if len(arr) == 0 or (shape is not None and arr.shape[1:] != shape):
        raise ValueError(f"{group.name}/{name} shape 非法: {arr.shape}")
    if finite and not np.isfinite(arr).all():
        raise ValueError(f"{group.name}/{name} 有非有限值")
    return arr


def timestamps(group, name="available_host_ts_ns"):
    arr = checked_series(group, name, ())
    if arr.dtype.kind not in "iu" or (arr <= 0).any() or (np.diff(arr) < 0).any():
        raise ValueError(f"{group.name}/{name} 必须为正整数且单调")
    return arr.astype(np.int64)


def audit_episode(path, rules):
    path = Path(path).resolve()
    report = {"episode": path.name, "path": str(path), "status": "REJECT", "reasons": []}
    try:
        with h5py.File(path, "r") as f:
            if f.attrs.get("schema_version") not in ("piper_raw_v1", "piper_raw_v2") or not f.attrs.get(
                "complete", False
            ):
                raise ValueError("文件未完整关闭或 schema 不支持")
            if f.attrs.get("collection_error") or f.attrs.get("dropped_samples", 0):
                raise ValueError("采集阶段发生错误或队列丢样")
            if not str(f.attrs.get("task_instruction", "")).strip():
                raise ValueError("缺少任务语言指令")
            start, end = int(f.attrs["start_host_ts_ns"]), int(f.attrs["end_host_ts_ns"])
            duration = (end - start) / 1e9
            report["duration_s"] = duration
            reasons = report["reasons"]
            if not rules["min_duration_s"] <= duration <= rules["max_duration_s"]:
                reasons.append("duration_out_of_range")
            if end <= start:
                raise ValueError("Episode 时间范围非法")
            validate_images(f, decode=True)
            grid = np.arange(start, end, 1_000_000_000 // rules["fps"], dtype=np.int64)
            yellow = np.zeros(len(grid), dtype=bool)
            reused = np.zeros(len(grid), dtype=bool)
            indices, ages, selected_ts = {}, {}, {}
            for stream in IMAGE_STREAMS:
                group = f[stream]
                ts = timestamps(group)
                fid = checked_series(group, "frame_id", ())
                if len(fid) != len(ts):
                    raise ValueError(f"{stream} frame_id 长度不一致")
                # An SDK cache frame is available at its FIRST retrieval, never refreshed by polling.
                _, unique = np.unique(fid, return_index=True)
                valid = np.zeros(len(fid), dtype=bool)
                valid[unique] = True
                index, age = causal_indices(ts, grid, valid)
                indices[stream] = index.tolist()
                ages[stream] = age
                selected_ts[stream] = ts[index]
                prefix = "rgb" if stream.startswith("cameras/") else "tactile"
                if np.any(age > rules[prefix + "_red_ms"] * 1e6):
                    reasons.append(stream + "_age_red")
                yellow |= age > rules[prefix + "_green_ms"] * 1e6
                reused[1:] |= index[1:] == index[:-1]
                report[stream.replace("/", "_") + "_max_age_ms"] = float(age.max() / 1e6)
            skew = np.abs(selected_ts["tactile/left_raw"] - selected_ts["tactile/right_raw"])
            if np.any(skew > rules["tactile_skew_red_ms"] * 1e6):
                reasons.append("tactile_skew_red")
            yellow |= skew > rules["tactile_skew_green_ms"] * 1e6
            report["tactile_max_skew_ms"] = float(skew.max() / 1e6)

            feedback = f["robot_feedback"]
            ts = timestamps(feedback)
            joint = checked_series(feedback, "joint_pos", (6,))
            grip = checked_series(feedback, "gripper_pos", (1,))[:, 0]
            tcp = checked_series(feedback, "tcp_pose", (6,))
            new = checked_series(feedback, "is_new_packet", ()).astype(bool)
            if any(len(a) != len(ts) for a in (joint, grip, tcp, new)):
                raise ValueError("机器人反馈长度不一致")
            index, _ = causal_indices(ts, grid, new)
            indices["robot_feedback"] = index.tolist()
            robot_age = np.zeros(len(grid), dtype=np.int64)
            for component in ("joint", "tcp", "gripper"):
                avail = timestamps(feedback, component + "_available_host_ts_ns")
                sdk = checked_series(feedback, "sdk_" + component + "_ts", ())
                if (
                    len(avail) != len(ts)
                    or len(sdk) != len(ts)
                    or (avail > ts).any()
                    or (np.diff(sdk) < 0).any()
                ):
                    raise ValueError(f"{component} 反馈时间非法")
                if np.any((np.diff(sdk) == 0) & (np.diff(avail) != 0)):
                    raise ValueError(f"{component} 缓存包被重新打点")
                robot_age = np.maximum(robot_age, grid - avail[index])
            if np.any(robot_age > rules["robot_red_ms"] * 1e6):
                reasons.append("robot_age_red")
            report["robot_max_age_ms"] = float(robot_age.max() / 1e6)

            command = f["robot_command"]
            cmd_ts = timestamps(command, "cmd_call_begin_ns")
            cmd_end = timestamps(command, "cmd_call_end_ns")
            cmd_seq = checked_series(command, "cmd_seq", ())
            targets = checked_series(command, "target_joint_pos", (6,))
            target_grip = checked_series(command, "target_gripper_pos", (1,))[:, 0]
            target_tcp = checked_series(command, "target_tcp_pose", (6,))
            if any(
                len(a) != len(cmd_ts) for a in (cmd_end, cmd_seq, targets, target_grip, target_tcp)
            ):
                raise ValueError("指令字段长度不一致")
            if (cmd_end < cmd_ts).any() or (np.diff(cmd_seq) <= 0).any():
                raise ValueError("指令时序或序号错误")
            cmd_index, held = action_indices(cmd_ts, grid, int(rules["action_window_ms"] * 1e6))
            indices["robot_command"] = cmd_index.tolist()
            holds = grid - cmd_ts[cmd_index]
            for k in np.flatnonzero(held & (holds > rules["action_max_hold_ms"] * 1e6)):
                # A long hold is allowed only with measured stable feedback at the held target.
                lookback = int(rules["stationary_lookback_s"] * 1e9)
                lower = np.searchsorted(ts, grid[k] - lookback, side="right") - 1
                upper = index[k] + 1
                stationary = lower >= 0 and upper > lower
                if stationary:
                    q = joint[lower:upper]
                    g = grip[lower:upper]
                    stationary = (
                        np.max(np.ptp(q, axis=0)) <= rules["stationary_joint_tolerance_rad"]
                        and np.ptp(g) <= rules["stationary_gripper_tolerance_m"]
                        and np.max(np.abs(q[-1] - targets[cmd_index[k]]))
                        <= rules["stationary_joint_tolerance_rad"]
                        and abs(g[-1] - target_grip[cmd_index[k]])
                        <= rules["stationary_gripper_tolerance_m"]
                    )
                if not stationary:
                    reasons.append("action_communication_gap")
                    break
            report["zoh_ratio"] = float(held.mean())
            report["max_hold_ms"] = float(np.maximum(holds, 0).max() / 1e6)
            report["yellow_ratio"] = float(yellow.mean())
            if yellow.mean() > rules["max_yellow_ratio"]:
                reasons.append("yellow_ratio_exceeded")
            if np.any(yellow[1:] & yellow[:-1]):
                reasons.append("consecutive_yellow_steps")

            contact = np.zeros(len(grid), dtype=bool)
            for values, times in ((target_grip, cmd_ts), (grip, ts)):
                anchor = values[0]
                for value, time_ns in zip(values[1:], times[1:]):
                    if abs(value - anchor) >= rules["gripper_change_m"]:
                        contact |= np.abs(grid - time_ns) <= rules["contact_window_s"] * 1e9
                        anchor = value
            if np.any(contact & (yellow | reused)):
                reasons.append("contact_window_frame_reuse_or_yellow")
            aligned_grip = grip[index]
            state = 0
            for value in aligned_grip:
                if state == 0 and value >= rules["open_threshold_m"]:
                    state = 1
                elif state == 1 and value <= rules["closed_threshold_m"]:
                    state = 2
                elif state == 2 and value >= rules["open_threshold_m"]:
                    state = 3
            report["gripper_cycle_complete"] = state == 3
            if rules["require_gripper_cycle"] and state != 3:
                reasons.append("missing_open_close_open_cycle")
            if not reasons:
                report["status"] = "PASS"
            sync = {
                "schema_version": "piper_sync_v1",
                "source_path": str(path),
                "source_sha256": fingerprint(path),
                "fps": rules["fps"],
                "grid_host_ts_ns": grid.tolist(),
                "indices": indices,
                "rules": rules,
                "automatic_status": report["status"],
            }
            report["source_sha256"] = sync["source_sha256"]
            return report, sync if report["status"] == "PASS" else None
    except (KeyError, ValueError, OSError, TypeError, IndexError, FFmpegError) as exc:
        report["reasons"].append(f"invalid_episode: {exc}")
        return report, None


def build_manifest(audit_dir, rules):
    audit_dir = Path(audit_dir).resolve()
    reports = json.loads((audit_dir / "audit_report.json").read_text(encoding="utf-8"))
    review_file = audit_dir / "review_decisions.json"
    reviews = json.loads(review_file.read_text(encoding="utf-8")) if review_file.exists() else {}
    eligible = []
    for report in reports:
        decision = reviews.get(report["episode"], {})
        if (
            report["status"] != "PASS"
            or decision.get("decision") != "approve"
            or decision.get("source_sha256") != report.get("source_sha256")
        ):
            continue
        sync_path = audit_dir / "sync_indices" / (Path(report["episode"]).stem + "_sync.json")
        if not sync_path.exists():
            continue
        eligible.append(
            {
                "path": report["path"],
                "sync_path": str(sync_path),
                "source_sha256": report["source_sha256"],
                "sync_sha256": fingerprint(sync_path),
                "split": "train",
            }
        )
    fraction = rules["val_fraction"]
    if not 0 <= fraction < 1:
        raise ValueError("val_fraction 必须位于 [0,1)")
    count = (
        min(len(eligible) - 1, max(1, round(len(eligible) * fraction)))
        if len(eligible) > 1 and fraction
        else 0
    )
    rng = np.random.default_rng(rules["split_seed"])
    for i in rng.permutation(len(eligible))[:count]:
        eligible[i]["split"] = "val"
    manifest = {
        "schema_version": "piper_manifest_v1",
        "fps": rules["fps"],
        "split_seed": rules["split_seed"],
        "episodes": eligible,
    }
    atomic_json(audit_dir / "approved_episodes.json", manifest)
    return manifest


def audit_batch(raw_dir, audit_dir, rules):
    audit_dir = Path(audit_dir)
    sync_dir = audit_dir / "sync_indices"
    sync_dir.mkdir(parents=True, exist_ok=True)
    reports = []
    for path in sorted(Path(raw_dir).glob("episode_*.h5")):
        report, sync = audit_episode(path, rules)
        reports.append(report)
        sync_path = sync_dir / (path.stem + "_sync.json")
        if sync:
            atomic_json(sync_path, sync)
        else:
            sync_path.unlink(missing_ok=True)
    for path in sorted(Path(raw_dir).glob("*.h5.partial")):
        reports.append(
            {
                "episode": path.name,
                "path": str(path.resolve()),
                "status": "REJECT",
                "reasons": ["incomplete_partial_file"],
            }
        )
    atomic_json(audit_dir / "audit_report.json", reports)
    fields = (
        sorted(set().union(*(r.keys() for r in reports)))
        if reports
        else ["episode", "status", "reasons"]
    )
    with (audit_dir / "audit_report.csv").open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for report in reports:
            writer.writerow({**report, "reasons": "; ".join(report["reasons"])})
    return reports, build_manifest(audit_dir, rules)
