"""Dependency-light encoding contract for stamped atomic IK results."""

from __future__ import annotations

import math


COLLISION_FIELD = "__self_collision__"
PATH_START_PREFIX = "__path_start_"


def path_start_field(joint_name):
    """Return the reserved result field carrying one path-start joint."""
    return f"{PATH_START_PREFIX}{joint_name}__"


def encode_correlated_result(
    joint_names,
    joint_positions,
    path_start_positions,
    collision,
):
    """Encode path start, solution, and collision in one stamped message."""
    names = tuple(str(name) for name in joint_names)
    positions = tuple(float(value) for value in joint_positions)
    path_start = tuple(float(value) for value in path_start_positions)
    reserved = tuple(path_start_field(name) for name in names)
    if not names or len(names) != len(positions) or len(names) != len(path_start):
        raise ValueError("IK names, solution, and path start must have equal length")
    if (
        len(set(names)) != len(names)
        or COLLISION_FIELD in names
        or any(name.startswith(PATH_START_PREFIX) for name in names)
    ):
        raise ValueError("IK joint names collide with reserved result fields")
    if any(not name for name in names) or not all(
        math.isfinite(value) for value in (*positions, *path_start)
    ):
        raise ValueError("IK names must be non-empty and joint values finite")
    return (
        (*names, *reserved, COLLISION_FIELD),
        (*positions, *path_start, 1.0 if bool(collision) else 0.0),
    )


def decode_correlated_result(names, positions):
    """Decode a complete atomic path result, rejecting malformed fields."""
    encoded_names = tuple(str(name) for name in names)
    encoded_positions = tuple(float(value) for value in positions)
    joint_count = (len(encoded_names) - 1) // 2
    joint_names = encoded_names[:joint_count]
    expected_start_names = tuple(path_start_field(name) for name in joint_names)
    if (
        len(encoded_names) < 3
        or len(encoded_names) % 2 != 1
        or len(encoded_names) != len(encoded_positions)
        or encoded_names[-1] != COLLISION_FIELD
        or len(set(encoded_names)) != len(encoded_names)
        or encoded_names[joint_count:-1] != expected_start_names
        or any(name.startswith(PATH_START_PREFIX) for name in joint_names)
        or not all(math.isfinite(value) for value in encoded_positions)
        or encoded_positions[-1] not in (0.0, 1.0)
    ):
        raise ValueError("malformed correlated IK result")
    return (
        joint_names,
        encoded_positions[:joint_count],
        encoded_positions[joint_count:-1],
        bool(encoded_positions[-1]),
    )
