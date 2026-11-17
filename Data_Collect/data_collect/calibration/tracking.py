"""Measured command-to-feedback timing, without rewriting state sample times."""

import json
import math

import numpy as np

from data_collect.common.config import fingerprint, local_path

CHANNELS = tuple(f"joint{i}" for i in range(1, 7)) + ("gripper",)
MEASUREMENT_KIND = "SDK command to host-received feedback effective tracking lag"


def validate_tracking_settings(timing, cfg):
    settings = timing["settings"]
    current = {
        "arm_type": cfg["arm_type"], "control_hz": cfg["control_hz"],
        "sdk_joint_method": "move_js" if cfg["teleop"]["fast_mode"] else "move_j",
        "speed_percent": cfg["teleop"]["speed_percent"],
        "tcp_offset": list(cfg["tcp_offset"]),
        "urdf_sha256": fingerprint(local_path(cfg["urdf"])),
    }
    if settings != current:
        changed = [name for name in current if current[name] != settings.get(name)]
        raise ValueError(f"机械臂延迟标定与当前设置不符: {', '.join(changed)}；请恢复标定工况或重新测量")


def load_tracking_latency(cfg):
    path = cfg.get("robot_tracking_latency_file")
    if not path:
        return None
    path = local_path(path)
    profile = json.loads(path.read_text(encoding="utf-8"))
    if (profile.get("schema_version") != "piper_tracking_latency_v1"
            or profile.get("measurement_kind") != MEASUREMENT_KIND
            or profile.get("measurement_completed") is not True
            or profile.get("unit") != "ms" or set(profile.get("channels", {})) != set(CHANNELS)):
        raise ValueError("机械臂延迟文件需要完整的六关节和夹爪实测结果")
    delays = []
    for name in CHANNELS:
        row = profile["channels"][name]
        value = row.get("delay_ms")
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0
                or row.get("at_search_boundary") is not False
                or row.get("complete") is not True or row.get("dropped_samples") != 0):
            raise ValueError(f"{name} 缺少有效跟随延迟测量")
        delays.append(int(round(value * 1e6)))
    timing = {
        "profile_file": str(path), "profile_sha256": fingerprint(path),
        "latency_ns": delays, "channels": list(CHANNELS),
        "semantics": "estimated_command_to_feedback_response; not state sample-time correction",
        "settings": {name: profile[name] for name in
                     ("arm_type", "control_hz", "sdk_joint_method", "speed_percent", "tcp_offset", "urdf_sha256")},
    }
    validate_tracking_settings(timing, cfg)
    cfg["robot_tracking_latency"] = timing
    return timing


def response_fields(joint_begin_ns, gripper_begin_ns, timing):
    calls = np.array([joint_begin_ns] * 6 + [gripper_begin_ns], dtype=np.int64)
    return {
        "joint_cmd_call_begin_ns": np.int64(joint_begin_ns),
        "gripper_cmd_call_begin_ns": np.int64(gripper_begin_ns),
        "expected_feedback_host_ts_ns": calls + np.asarray(timing["latency_ns"], dtype=np.int64),
    }


def recorded_tracking(file):
    timing = json.loads(file.attrs.get("config", "{}")).get("robot_tracking_latency")
    command = file["robot_command"]
    keys = ("joint_cmd_call_begin_ns", "gripper_cmd_call_begin_ns", "expected_feedback_host_ts_ns")
    if not timing:
        if any(key in command for key in keys):
            raise ValueError("机器人预计响应时间缺少随录制保存的标定快照")
        return None
    delays = np.asarray(timing["latency_ns"])
    if (timing.get("channels") != list(CHANNELS) or not timing.get("profile_sha256")
            or delays.shape != (7,) or delays.dtype.kind not in "iu" or (delays < 0).any()):
        raise ValueError("机器人跟随延迟标定快照非法")
    begin = command["cmd_call_begin_ns"][:]
    joint, grip, expected = (command[key][:] for key in keys)
    calls = np.column_stack([joint] * 6 + [grip])
    if (joint.shape != begin.shape or grip.shape != begin.shape or expected.shape != (len(begin), 7)
            or any(v.dtype.kind not in "iu" for v in (joint, grip, expected))
            or (calls <= 0).any() or (calls > begin[:, None]).any()
            or (np.diff(calls, axis=0) < 0).any()
            or not np.array_equal(expected, calls + delays)):
        raise ValueError("机器人预计响应时间与实际调用时间/标定不符")
    return timing


def response_reference_indices(file, grid, grid_segments, intervals):
    """Per-channel causal expected-response references, confined to each segment.

    -1 marks a segment's initial region without a recorded responding target.
    This diagnostic reference never replaces the SDK-send-time training action.
    """
    if recorded_tracking(file) is None:
        return None
    command = file["robot_command"]
    expected = command["expected_feedback_host_ts_ns"][:]
    begin = command["cmd_call_begin_ns"][:]
    calls = np.column_stack([command["joint_cmd_call_begin_ns"][:]] * 6
                            + [command["gripper_cmd_call_begin_ns"][:]])
    segments = (command["recording_segment"][:] if file.attrs.get("recording_mode") == "grip_paused"
                else np.zeros(len(begin), dtype=np.int64))
    result = np.full((len(grid), 7), -1, dtype=np.int64)
    grid, grid_segments = np.asarray(grid), np.asarray(grid_segments)
    for segment, (lower, upper) in enumerate(intervals):
        dest = np.flatnonzero(grid_segments == segment)
        for column in range(7):
            source = np.flatnonzero((segments == segment) & (begin < upper) & (calls[:, column] >= lower)
                                    & (expected[:, column] >= lower) & (expected[:, column] < upper))
            if not len(source):
                continue
            # A mixed command row also contains a held target for the other
            # component. Do not select a future row just because its held
            # component already has an earlier expected response time.
            available = np.maximum(expected[source, column], begin[source])
            pos = np.searchsorted(available, grid[dest], side="right") - 1
            valid = pos >= 0
            result[dest[valid], column] = source[pos[valid]]
    return result


def response_reference_values(file, indices):
    indices = np.asarray(indices, dtype=np.int64)
    valid = indices >= 0
    command = file["robot_command"]
    targets = np.column_stack((command["target_joint_pos"][:], command["target_gripper_pos"][:]))
    values = np.zeros(indices.shape, dtype=np.float32)
    rows, columns = np.nonzero(valid)
    values[rows, columns] = targets[indices[rows, columns], columns]
    return values, valid.astype(np.int64)
