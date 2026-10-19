"""ROS-side copy of the versioned Piper learning contract."""

from __future__ import annotations

import numpy as np


JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")
ACTION_DIM = len(JOINT_NAMES)
STATE_DIM = ACTION_DIM * 2
JOINT_LOWER = np.asarray(
    [-2.618, 0.0, -2.9670597, -1.7453292, -1.2217304, -2.0943951, 0.0], dtype=np.float32
)
JOINT_UPPER = np.asarray(
    [2.168, 3.1415926, 0.0, 1.7453292, 1.2217304, 2.0943951, 0.07], dtype=np.float32
)
MAX_DELTA_PER_TICK = np.asarray([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.002], dtype=np.float32)
MAX_TRACKING_ERROR = np.asarray(
    [0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.02], dtype=np.float32
)


def cameras_ready(
    now: float,
    arrivals: dict[str, float | None],
    timestamps: dict[str, float | None],
    timeout_s: float,
    max_skew_s: float,
) -> bool:
    """Return whether both policy views are fresh and mutually aligned."""
    if timeout_s <= 0.0 or max_skew_s <= 0.0:
        return False
    roles = ("wrist", "global")
    if any(arrivals.get(role) is None or timestamps.get(role) is None for role in roles):
        return False
    return bool(
        all(0.0 <= now - float(arrivals[role]) <= timeout_s for role in roles)
        and abs(float(timestamps["wrist"]) - float(timestamps["global"])) <= max_skew_s
    )


def state_by_name(names, positions, velocities) -> np.ndarray | None:
    """Build canonical positions then velocities from a complete JointState."""
    if len(names) != len(positions) or len(names) != len(velocities):
        return None
    values = {
        str(name): (float(position), float(velocity))
        for name, position, velocity in zip(names, positions, velocities)
    }
    try:
        state = np.asarray(
            [
                *[values[name][0] for name in JOINT_NAMES],
                *[values[name][1] for name in JOINT_NAMES],
            ],
            dtype=np.float32,
        )
    except KeyError:
        return None
    return state if np.isfinite(state).all() else None


def complete_action_by_name(names, positions) -> np.ndarray | None:
    """Build a finite canonical action only when all seven names are present."""
    if len(names) != len(positions):
        return None
    values = {str(name): float(position) for name, position in zip(names, positions)}
    try:
        action = np.asarray([values[name] for name in JOINT_NAMES], dtype=np.float32)
    except KeyError:
        return None
    return action if np.isfinite(action).all() else None
