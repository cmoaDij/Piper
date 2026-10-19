from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from ik_safety_contract import (  # noqa: E402
    canonical_joint_positions,
    feedback_aware_joint_limits,
    feedback_freshness_error,
    joint_path_samples,
    pose_residual,
    resolve_locked_joint_names,
    select_best_pose_candidate,
)


def test_feedback_freshness_requires_source_and_arrival_times():
    assert feedback_freshness_error(1.0, 1.0, 1.1, 0.15) is None
    assert "stale" in feedback_freshness_error(1.0, 1.19, 1.2, 0.15)
    assert "stale" in feedback_freshness_error(1.19, 1.0, 1.2, 0.15)
    assert "future-dated" in feedback_freshness_error(1.21, 1.2, 1.2, 0.15)


def test_feedback_is_reordered_by_canonical_joint_name():
    result = canonical_joint_positions(
        ("gripper", "joint2", "joint1"),
        (0.07, 0.2, 0.1),
        ("joint1", "joint2"),
    )
    assert np.allclose(result, (0.1, 0.2))
    assert canonical_joint_positions(
        ("joint1", "joint1"), (0.0, 0.0), ("joint1",)
    ) is None
    assert canonical_joint_positions(
        ("joint1",), (float("nan"),), ("joint1",)
    ) is None


def test_non_commanded_gripper_joints_are_automatically_locked():
    locked = resolve_locked_joint_names(
        (
            "universe",
            "joint1",
            "joint2",
            "gripper",
            "gripper_joint1",
            "gripper_joint2",
        ),
        ("gripper_joint1", "gripper_joint2"),
        ("joint1", "joint2"),
    )
    assert locked == ["gripper", "gripper_joint1", "gripper_joint2"]


def test_joint_contract_rejects_missing_duplicate_or_locked_outputs():
    model_names = ("universe", "joint1", "joint2", "gripper")
    with pytest.raises(ValueError, match="absent"):
        resolve_locked_joint_names(model_names, (), ("joint1", "joint3"))
    with pytest.raises(ValueError, match="duplicates"):
        resolve_locked_joint_names(model_names, (), ("joint1", "joint1"))
    with pytest.raises(ValueError, match="also be configured as locked"):
        resolve_locked_joint_names(model_names, ("joint1",), ("joint1", "joint2"))


def test_pose_residual_reports_translation_and_shortest_rotation():
    target = np.eye(4)
    achieved = np.eye(4)
    achieved[0, 3] = 0.001
    achieved[:3, :3] = np.asarray(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    translation, rotation = pose_residual(target, achieved)
    assert translation == pytest.approx(0.001)
    assert rotation == pytest.approx(np.pi / 2.0)


def test_best_pose_candidate_keeps_previous_when_solver_is_worse():
    target = np.eye(4)
    previous = np.eye(4)
    solved = np.eye(4)
    previous[0, 3] = 0.001
    solved[0, 3] = 0.004
    name, joints, position, rotation = select_best_pose_candidate(
        target,
        (
            ("previous", (1.0,), previous),
            ("solver", (2.0,), solved),
        ),
        0.0015,
        0.02,
    )
    assert name == "previous"
    assert np.allclose(joints, (1.0,))
    assert position == pytest.approx(0.001)
    assert rotation == pytest.approx(0.0)


def test_best_pose_candidate_can_choose_rotation_improvement():
    target = np.eye(4)
    previous = np.eye(4)
    solved = np.eye(4)
    previous[:3, :3] = np.asarray(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    solved[0, 3] = 0.001
    name, _, _, _ = select_best_pose_candidate(
        target,
        (("previous", (1.0,), previous), ("solver", (2.0,), solved)),
        0.0015,
        0.02,
    )
    assert name == "solver"


def test_feedback_limit_overrun_is_included_but_cannot_expand_farther():
    lower, upper = feedback_aware_joint_limits(
        (0.0, -1.0),
        (1.0, 1.0),
        (-0.057, 0.25),
        0.1,
    )
    assert np.allclose(lower, (-0.057, -1.0))
    assert np.allclose(upper, (1.0, 1.0))
    with pytest.raises(ValueError, match="exceeds"):
        feedback_aware_joint_limits((0.0,), (1.0,), (-0.11,), 0.1)


def test_joint_path_samples_cover_endpoints_and_bound_every_step():
    samples = joint_path_samples((0.0, 0.0), (0.11, -0.04), 0.05)
    assert np.allclose(samples[0], (0.0, 0.0))
    assert np.allclose(samples[-1], (0.11, -0.04))
    assert all(
        np.max(np.abs(current - previous)) <= 0.05 + 1e-12
        for previous, current in zip(samples, samples[1:])
    )


@pytest.mark.parametrize("step", (0.0, -0.1, float("nan")))
def test_joint_path_rejects_invalid_step(step):
    with pytest.raises(ValueError):
        joint_path_samples((0.0,), (1.0,), step)
