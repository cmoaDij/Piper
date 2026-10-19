import numpy as np
import pytest

from data_collect.common.diagnostics import require_feedback_fresh
from data_collect.control.motion import MotionLimiter, check_feedback_motion


def limiter(config):
    instance = MotionLimiter({f"joint{i}": (-2.0, 2.0) for i in range(1, 7)}, config["motion"])
    instance.reset(np.zeros(6), 1_000_000_000)
    return instance


def test_trajectory_respects_speed_acceleration_and_command_step(config):
    instance = limiter(config)
    previous, velocity = np.zeros(6), np.zeros(6)
    for i in range(1, 151):
        position = np.asarray(instance.step(np.full(6, .04), previous, 1_000_000_000 + i * 20_000_000))
        speed = (position - previous) / .02
        assert np.max(np.abs(speed)) <= config["motion"]["max_velocity_rad_s"] + 1e-9
        assert np.max(np.abs(speed - velocity)) <= config["motion"]["max_acceleration_rad_s2"] * .02 + 1e-9
        assert np.max(np.abs(position - previous)) <= config["motion"]["max_command_step_rad"] + 1e-9
        previous, velocity = position, speed
    assert np.allclose(previous, .04, atol=.001)


def test_control_gap_and_first_target_jump_are_rejected(config):
    instance = limiter(config)
    with pytest.raises(ValueError, match="首条目标跳变"):
        instance.check_first(np.full(6, .1), np.zeros(6))
    with pytest.raises(ValueError, match="控制周期中断"):
        instance.step(np.zeros(6), np.zeros(6), 1_100_000_000)


def test_back_and_forth_feedback_cannot_cancel_travel(config):
    history = [{"joint_available_host_ts_ns": 1_000_000_000 + i * 20_000_000,
                "joint_pos": np.full(6, np.deg2rad(.6) if i % 2 else 0)} for i in range(6)]
    with pytest.raises(ValueError, match="累计变化"):
        check_feedback_motion(history, config["motion"])


def test_stale_missing_or_future_feedback_cannot_pass():
    now = 1_000_000_000
    fresh = {key + "_available_host_ts_ns": now for key in ("joint", "tcp", "gripper")}
    require_feedback_fresh(fresh, now, 25)
    for changed in ({}, {**fresh, "joint_available_host_ts_ns": now - 26_000_000},
                    {**fresh, "tcp_available_host_ts_ns": now + 1}):
        with pytest.raises(ValueError):
            require_feedback_fresh(changed, now, 25)
