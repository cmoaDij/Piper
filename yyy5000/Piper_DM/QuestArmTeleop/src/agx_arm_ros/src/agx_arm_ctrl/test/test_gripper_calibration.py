"""Tests for fail-closed gripper zero-calibration preconditions."""

from types import SimpleNamespace

from agx_arm_ctrl.gripper_calibration import (
    calibrated_zero_feedback_failure,
    calibration_precondition_failure,
)


def status(width=-0.0056, enabled=True, **faults):
    values = {
        "width": width,
        "driver_enable_status": enabled,
        "voltage_too_low": False,
        "motor_overheating": False,
        "driver_overcurrent": False,
        "driver_overheating": False,
        "sensor_status": False,
        "driver_error_status": False,
        "homing_status": False,
    }
    values.update(faults)
    return SimpleNamespace(**values)


def check(**overrides):
    arguments = {
        "control_enabled": False,
        "arm_positions": [0.0] * 6,
        "home_tolerance": 0.03,
        "gripper_status": status(),
        "closed_tolerance": 0.01,
    }
    arguments.update(overrides)
    return calibration_precondition_failure(**arguments)


def test_closed_enabled_fault_free_gripper_at_home_is_accepted():
    assert check() is None


def test_open_control_gate_or_non_home_arm_is_rejected():
    assert "gate" in check(control_enabled=True)
    assert "not at verified zero home" in check(
        arm_positions=[0.0, 0.0, 0.0, 0.04, 0.0, 0.0]
    )


def test_disabled_gripper_can_enter_the_official_calibration_sequence():
    assert check(gripper_status=status(enabled=False)) is None


def test_faulted_or_not_closed_gripper_is_rejected():
    assert "faults" in check(gripper_status=status(driver_overcurrent=True))
    assert "not near" in check(gripper_status=status(width=0.02))


def test_postcheck_requires_enabled_homed_zero_feedback():
    assert calibrated_zero_feedback_failure(
        gripper_status=status(width=0.0, enabled=True, homing_status=True),
        closed_tolerance=0.01,
    ) is None
    assert "homing status" in calibrated_zero_feedback_failure(
        gripper_status=status(width=0.0, enabled=True, homing_status=False),
        closed_tolerance=0.01,
    )


def test_postcheck_still_rejects_disabled_faulted_or_nonzero_feedback():
    assert "not enabled" in calibrated_zero_feedback_failure(
        gripper_status=status(enabled=False),
        closed_tolerance=0.01,
    )
    assert "faults" in calibrated_zero_feedback_failure(
        gripper_status=status(driver_overcurrent=True),
        closed_tolerance=0.01,
    )
    assert "not stable at zero" in calibrated_zero_feedback_failure(
        gripper_status=status(width=0.02, homing_status=True),
        closed_tolerance=0.01,
    )
