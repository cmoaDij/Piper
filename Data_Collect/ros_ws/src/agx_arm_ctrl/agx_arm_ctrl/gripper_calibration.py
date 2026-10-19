"""Pure safety checks for one-shot closed-gripper zero calibration."""

from __future__ import annotations

import math
from typing import Sequence


GRIPPER_FAULT_FIELDS = (
    "voltage_too_low",
    "motor_overheating",
    "driver_overcurrent",
    "driver_overheating",
    "sensor_status",
    "driver_error_status",
)


def calibration_precondition_failure(
    *,
    control_enabled: bool,
    arm_positions: Sequence[float] | None,
    home_tolerance: float,
    gripper_status,
    closed_tolerance: float,
) -> str | None:
    """Return the first fail-closed reason, or ``None`` when safe."""
    if control_enabled:
        return "external control gate must be closed"
    if arm_positions is None or len(arm_positions) == 0:
        return "complete arm position feedback is unavailable"
    positions = [float(value) for value in arm_positions]
    if not all(math.isfinite(value) for value in positions):
        return "arm position feedback is non-finite"
    maximum = max(abs(value) for value in positions)
    if maximum > float(home_tolerance):
        return (
            "arm is not at verified zero home: "
            f"max_abs_position={maximum:.6f} rad"
        )
    if gripper_status is None:
        return "gripper feedback is unavailable"
    active_faults = [
        field
        for field in GRIPPER_FAULT_FIELDS
        if bool(getattr(gripper_status, field))
    ]
    if active_faults:
        return "gripper reports faults: " + ", ".join(active_faults)
    width = float(gripper_status.width)
    if not math.isfinite(width):
        return "gripper width feedback is non-finite"
    if abs(width) > float(closed_tolerance):
        return (
            "gripper feedback is not near the physically closed position: "
            f"width={width:.6f} m"
        )
    return None


def calibrated_zero_feedback_failure(
    *,
    gripper_status,
    closed_tolerance: float,
) -> str | None:
    """Validate strict post-calibration feedback."""
    if gripper_status is None:
        return "gripper feedback is unavailable"
    if not bool(gripper_status.driver_enable_status):
        return "gripper driver is not enabled"
    active_faults = [
        field
        for field in GRIPPER_FAULT_FIELDS
        if bool(getattr(gripper_status, field))
    ]
    if active_faults:
        return "gripper reports faults: " + ", ".join(active_faults)
    if not bool(gripper_status.homing_status):
        return "gripper homing status is not asserted"
    width = float(gripper_status.width)
    if not math.isfinite(width):
        return "gripper width feedback is non-finite"
    if abs(width) > float(closed_tolerance):
        return f"gripper feedback is not stable at zero: width={width:.6f} m"
    return None
