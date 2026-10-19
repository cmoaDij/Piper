"""Dependency-free Quest trigger to gripper position mapping."""

import math
from typing import Any


def gripper_position_from_trigger(
    trigger_value: Any, gripper_max_range: float
) -> float:
    """Map a released trigger to an open gripper and a pressed trigger to closed."""
    try:
        normalized_trigger = float(trigger_value)
    except (TypeError, ValueError):
        normalized_trigger = 0.0

    if not math.isfinite(normalized_trigger):
        normalized_trigger = 0.0
    normalized_trigger = max(0.0, min(normalized_trigger, 1.0))
    return (1.0 - normalized_trigger) * gripper_max_range
