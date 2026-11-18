import copy
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from data_collect.control.quest_world import (
    PROTOCOL, WorldInputState, decode_packet, heading_to_base, pose_matrix,
)


def packet(seq=1, generation=1, **overrides):
    return {"protocol": PROTOCOL, "space": "LOCAL", "session": 123,
            "generation": generation, "seq": seq, "device_monotonic_ns": 12345,
            "host_monotonic_ns": time.monotonic_ns(), "valid": True,
            "grip": [.2, 1.1, -.4, 0., 0., 0., 1.],
            "aim": [.2, 1.1, -.4, 0., 0., 0., 1.],
            "buttons": {"A": False, "B": False, "RJ": False, "RG": False, "rightTrig": 0.},
            **overrides}


@pytest.mark.parametrize("yaw", [0., .8, -2.4])
def test_heading_maps_forward_left_up_to_base_axes_and_stays_right_handed(yaw):
    aim = pose_matrix([0., 0., 0., *Rotation.from_euler("y", yaw).as_quat()])
    mapping = heading_to_base(aim)
    forward = -aim[:3, 2]
    up = np.array([0., 1., 0.])
    np.testing.assert_allclose(mapping @ np.stack((forward, np.cross(up, forward), up), axis=1), np.eye(3), atol=1e-12)
    assert np.linalg.det(mapping) == pytest.approx(1.)


def test_heading_uses_horizontal_projection_and_rejects_vertical_pointer():
    aim = pose_matrix([0., 0., 0., *Rotation.from_euler("xyz", [.4, .7, -.3]).as_quat()])
    mapping = heading_to_base(aim)
    np.testing.assert_allclose(mapping[2], [0., 1., 0.])
    vertical = pose_matrix([0., 0., 0., *Rotation.from_euler("x", np.pi/2).as_quat()])
    with pytest.raises(ValueError, match="竖直"):
        heading_to_base(vertical)


def test_world_direction_survives_grip_turns_and_rebasing_but_reset_requires_new_alignment():
    state = WorldInputState()
    state.update(packet())
    rotation = state.align_once().copy()
    before = state.mapped_pose()
    changed = packet(2, grip=[.2, 1.1, -.5, *Rotation.from_euler("xyz", [.4, -.7, 1.]).as_quat()],
                     aim=[.2, 1.1, -.5, *Rotation.from_euler("y", 1.5).as_quat()])
    state.update(changed)
    np.testing.assert_allclose(state.align_once(), rotation)
    np.testing.assert_allclose(state.mapped_pose()[:3, 3] - before[:3, 3], [.1, 0., 0.], atol=1e-12)
    assert state.update(packet(3, generation=2))
    assert state.rotation is None and state.mapped_pose() is None


def test_validity_and_stale_packets_cannot_be_used_for_control():
    state = WorldInputState()
    now = time.monotonic_ns()
    state.update(packet(host_monotonic_ns=now))
    state.align_once()
    assert not state.valid(now + 500_000_001)
    with pytest.raises(ValueError, match="过期"):
        state.update(packet(2, host_monotonic_ns=now), now + 500_000_001)
    state.update(packet(3, valid=False, grip=None, aim=None))
    assert not state.valid() and state.mapped_pose() is None
    assert state.rotation is not None  # Loss of tracking need not change LOCAL axes.


def test_wrong_frame_corrupt_pose_and_duplicate_sequence_rejected():
    for overrides in ({"protocol": "legacy"}, {"space": "VIEW"},
                      {"grip": [0., 0., 0., 0., 0., 0., 0.]},
                      {"grip": [float("nan"), 0., 0., 0., 0., 0., 1.]}):
        with pytest.raises(ValueError):
            decode_packet(packet(**overrides))
    state = WorldInputState()
    first = packet()
    state.update(first)
    duplicate = copy.deepcopy(first)
    duplicate["grip"][0] += 1.
    assert not state.update(duplicate)
    assert state.packet["grip"][0] == .2


@pytest.mark.parametrize("tracking,message", [
    ({"focused": False}, "输入焦点"),
    ({"focused": True, "grip_active": False}, "右手柄未激活"),
    ({"reference_ready": False}, "重置追踪空间"),
    ({"grip": False, "aim": False}, "完整空间追踪"),
    ({}, "位姿无效"),
])
def test_tracking_rejection_reports_the_specific_cause(tracking, message):
    state = WorldInputState()
    state.update(packet(valid=False, tracking=tracking))
    assert message in state.unavailable_reason()
    assert not state.valid()


def test_tracking_status_distinguishes_no_data_stale_and_ready():
    state = WorldInputState()
    assert "尚未收到" in state.unavailable_reason()
    now = time.monotonic_ns()
    state.update(packet(host_monotonic_ns=now), now)
    assert state.unavailable_reason(now) is None
    assert "500 ms" in state.unavailable_reason(now + 500_000_001)


@pytest.mark.parametrize("tracking", [{"focused": False}, {"focused": True, "grip_active": False}])
def test_system_boundary_prompt_is_included_in_input_recovery_guidance(tracking):
    state = WorldInputState()
    state.update(packet(valid=False, tracking=tracking))
    assert "游戏边界" in state.unavailable_reason()
    assert not state.valid()
