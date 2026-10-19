"""Pure validation helpers for correlated Piper IK requests and results."""

from __future__ import annotations

import math

import numpy as np


def feedback_freshness_error(
    source_timestamp: float,
    arrival_timestamp: float,
    now: float,
    timeout_s: float,
):
    """Validate both producer and local-arrival ages for IK feedback."""
    values = np.asarray(
        (source_timestamp, arrival_timestamp, now, timeout_s),
        dtype=np.float64,
    )
    if not np.isfinite(values).all() or timeout_s <= 0.0:
        return "feedback timestamps or timeout are invalid"
    source_age = float(now) - float(source_timestamp)
    arrival_age = float(now) - float(arrival_timestamp)
    if source_age < 0.0 or arrival_age < 0.0:
        return (
            "feedback is future-dated: "
            f"source_age={source_age:.6f}s, arrival_age={arrival_age:.6f}s"
        )
    if source_age > timeout_s or arrival_age > timeout_s:
        return (
            "feedback is stale: "
            f"source_age={source_age:.6f}s, arrival_age={arrival_age:.6f}s, "
            f"limit={timeout_s:.6f}s"
        )
    return None


def resolve_locked_joint_names(
    model_joint_names,
    configured_locked_joints,
    output_joint_names,
):
    """Return every movable model joint that must be excluded from IK.

    The commanded joint list is the authoritative IK degree-of-freedom
    contract.  This prevents an independently controlled gripper from being
    left in the optimization merely because its URDF joint name was omitted
    from ``locked_joints``.
    """
    model_names = tuple(name for name in model_joint_names if name != "universe")
    output_names = tuple(output_joint_names)
    if not output_names or any(not name for name in output_names):
        raise ValueError("output_joint_names must contain non-empty joint names")
    if len(set(output_names)) != len(output_names):
        raise ValueError("output_joint_names must not contain duplicates")

    missing = [name for name in output_names if name not in model_names]
    if missing:
        raise ValueError(f"output_joint_names are absent from the URDF: {missing}")

    configured = set(configured_locked_joints)
    overlap = [name for name in output_names if name in configured]
    if overlap:
        raise ValueError(
            f"commanded joints cannot also be configured as locked: {overlap}"
        )

    return [name for name in model_names if name not in output_names]


def canonical_joint_positions(names, positions, required_names):
    """Return required joints by name, rejecting partial or ambiguous feedback."""
    names = tuple(str(name) for name in names)
    positions = tuple(positions)
    required = tuple(str(name) for name in required_names)
    if (
        len(names) != len(positions)
        or len(set(names)) != len(names)
        or len(set(required)) != len(required)
        or not required
    ):
        return None
    try:
        values = {name: float(value) for name, value in zip(names, positions)}
        result = np.asarray([values[name] for name in required], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        return None
    return result if np.isfinite(result).all() else None


def pose_residual(target_pose, achieved_pose):
    """Return translation and shortest-angle rotation residuals for two poses."""
    target = np.asarray(target_pose, dtype=np.float64)
    achieved = np.asarray(achieved_pose, dtype=np.float64)
    for name, pose in (("target", target), ("achieved", achieved)):
        if (
            pose.shape != (4, 4)
            or not np.isfinite(pose).all()
            or not np.allclose(pose[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
            or not math.isclose(
                float(np.linalg.det(pose[:3, :3])), 1.0, abs_tol=1e-5
            )
        ):
            raise ValueError(f"{name} pose must be a finite rigid 4x4 transform")
    translation = float(np.linalg.norm(target[:3, 3] - achieved[:3, 3]))
    relative = target[:3, :3].T @ achieved[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return translation, float(math.acos(cosine))


def select_best_pose_candidate(
    target_pose,
    candidates,
    position_tolerance,
    rotation_tolerance,
):
    """Select the candidate with the smallest normalized 6-D pose residual."""
    position_limit = float(position_tolerance)
    rotation_limit = float(rotation_tolerance)
    if (
        not math.isfinite(position_limit)
        or not math.isfinite(rotation_limit)
        or position_limit <= 0.0
        or rotation_limit <= 0.0
    ):
        raise ValueError("pose candidate tolerances must be finite and positive")
    if not candidates:
        raise ValueError("at least one pose candidate is required")

    scored = []
    for name, joints, achieved_pose in candidates:
        position_error, rotation_error = pose_residual(
            target_pose, achieved_pose
        )
        score = (position_error / position_limit) ** 2 + (
            rotation_error / rotation_limit
        ) ** 2
        scored.append(
            (
                score,
                str(name),
                np.asarray(joints, dtype=np.float64).copy(),
                position_error,
                rotation_error,
            )
        )
    _, name, joints, position_error, rotation_error = min(
        scored, key=lambda item: item[0]
    )
    return name, joints, position_error, rotation_error


def feedback_aware_joint_limits(lower, upper, reference, margin):
    """Include a small feedback overrun without permitting farther motion."""
    lower = np.asarray(lower, dtype=np.float64)
    upper = np.asarray(upper, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    margin = float(margin)
    if (
        lower.ndim != 1
        or upper.shape != lower.shape
        or reference.shape != lower.shape
        or not np.isfinite(lower).all()
        or not np.isfinite(upper).all()
        or not np.isfinite(reference).all()
        or not math.isfinite(margin)
        or margin < 0.0
        or np.any(lower > upper)
    ):
        raise ValueError("joint limits, reference, and margin must be valid")
    effective_lower = np.maximum(lower - margin, np.minimum(lower, reference))
    effective_upper = np.minimum(upper + margin, np.maximum(upper, reference))
    if np.any(reference < effective_lower) or np.any(reference > effective_upper):
        raise ValueError("joint feedback exceeds the permitted limit margin")
    return effective_lower, effective_upper


def joint_path_samples(start, target, max_step_rad):
    """Sample a joint segment so every per-joint increment is bounded."""
    start = np.asarray(start, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    max_step = float(max_step_rad)
    if (
        start.ndim != 1
        or target.shape != start.shape
        or start.size == 0
        or not np.isfinite(start).all()
        or not np.isfinite(target).all()
        or not np.isfinite(max_step)
        or max_step <= 0.0
    ):
        raise ValueError("joint path needs equal finite vectors and a positive step")
    segments = max(1, int(math.ceil(float(np.max(np.abs(target - start))) / max_step)))
    return tuple(
        start + (target - start) * (index / segments)
        for index in range(segments + 1)
    )
