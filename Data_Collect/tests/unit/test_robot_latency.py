import json
import sys
import time
from unittest.mock import Mock

import h5py
import numpy as np
import pytest

from data_collect.application.collector import SensorRuntime
from data_collect.calibration.robot_latency import RobotLatencyManager


def config(tmp_path):
    return {
        "data_root": str(tmp_path), "cameras": {}, "tactile": {},
        "task_instruction": "robot latency", "operator": "test",
        "calibration_id": "timing_test", "queue_max_items": 32,
        "queue_max_mb": 1, "storage": {}, "audit": {"robot_red_ms": 25},
    }


def feedback(stamp):
    return {
        "available_host_ts_ns": np.int64(stamp),
        "joint_available_host_ts_ns": np.int64(stamp),
        "tcp_available_host_ts_ns": np.int64(stamp),
        "gripper_available_host_ts_ns": np.int64(stamp),
        "joint_pos": np.zeros(6, dtype=np.float32),
        "gripper_pos": np.array([.065], dtype=np.float32),
        "is_new_packet": np.bool_(True),
    }


def test_robot_recording_needs_no_images_or_encoder(tmp_path, monkeypatch):
    probe = Mock(side_effect=AssertionError("Image codec must not be probed"))
    monkeypatch.setattr("data_collect.recording.storage.select_encoder", probe)
    manager = RobotLatencyManager(config(tmp_path))
    manager.emit("robot_feedback", feedback(time.monotonic_ns()))
    path = manager.start()
    stamp = time.monotonic_ns()
    manager.emit("robot_command", {
        "cmd_call_begin_ns": np.int64(stamp), "cmd_call_end_ns": np.int64(stamp + 1),
        "target_joint_pos": np.zeros(6, dtype=np.float32),
        "target_gripper_pos": np.array([.065], dtype=np.float32),
    })
    manager.emit("robot_feedback", feedback(time.monotonic_ns()))
    manager.close()
    probe.assert_not_called()
    with h5py.File(path, "r") as file:
        assert file.attrs["complete"]
        assert file.attrs["schema_version"] == "piper_robot_latency_v1"
        assert file.attrs["not_for_training"]
        assert json.loads(file.attrs["image_streams"]) == []
        assert set(file) == {"robot_command", "robot_feedback"}
        assert len(file["robot_command/cmd_call_begin_ns"]) == 1
        assert len(file["robot_feedback/joint_pos"]) >= 1
    assert not list(tmp_path.rglob("*.mp4"))
    assert not path.with_suffix(".reserve").exists()


def test_robot_recording_preserves_feedback_freshness_gate(tmp_path):
    manager = RobotLatencyManager(config(tmp_path))
    with pytest.raises(RuntimeError, match="反馈尚未就绪"):
        manager.start()
    manager.emit("robot_feedback", feedback(time.monotonic_ns() - 1_000_000_000))
    with pytest.raises(ValueError, match="过期"):
        manager.start()
    assert not list(tmp_path.rglob("*.h5*"))


def test_robot_runtime_without_capture_does_not_import_image_devices(monkeypatch):
    monkeypatch.setitem(sys.modules, "data_collect.sensors.capture", None)
    manager = Mock()
    runtime = SensorRuntime({}, manager, None, None, None, capture_enabled=False)
    assert runtime.capture is None
    runtime.close()
