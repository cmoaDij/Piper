"""Unit tests for AGX gripper enable recovery."""

from types import SimpleNamespace

from agx_arm_ctrl.effector.agx_gripper import AgxGripperWrapper


class FakeGripperDriver:
    def __init__(self):
        self.enabled = False
        self.reset_calls = 0
        self.moves = []

    def get_gripper_status(self):
        foc_status = SimpleNamespace(
            voltage_too_low=False,
            motor_overheating=False,
            driver_overcurrent=False,
            driver_overheating=False,
            sensor_status=False,
            driver_error_status=False,
            driver_enable_status=self.enabled,
            homing_status=False,
        )
        message = SimpleNamespace(value=0.06272, force=0.0, foc_status=foc_status)
        return SimpleNamespace(msg=message, hz=200.0, timestamp=1.0)

    def reset_gripper(self):
        self.reset_calls += 1
        return True

    def move_gripper_m(self, value, force):
        self.moves.append((value, force))


class FakeArm:
    class OPTIONS:
        class EFFECTOR:
            AGX_GRIPPER = "agx_gripper"

    def __init__(self, gripper):
        self.gripper = gripper

    def init_effector(self, _effector_type):
        return self.gripper


def test_disabled_gripper_is_cleared_before_enable_width_command():
    driver = FakeGripperDriver()
    wrapper = AgxGripperWrapper(FakeArm(driver))
    assert wrapper.initialize()

    assert wrapper.move(width=0.04, force=1.0)
    assert driver.reset_calls == 1
    assert driver.moves == [(0.04, 1.0)]

    # A high-rate command stream must not repeatedly clear/disable the gripper.
    assert wrapper.move(width=0.03, force=1.0)
    assert driver.reset_calls == 1
    assert driver.moves[-1] == (0.03, 1.0)

    driver.enabled = True
    assert wrapper.move(width=0.02, force=1.0)
    assert driver.reset_calls == 1
    assert driver.moves[-1] == (0.02, 1.0)


def test_post_calibration_enable_does_not_insert_another_reset():
    driver = FakeGripperDriver()
    wrapper = AgxGripperWrapper(FakeArm(driver))
    assert wrapper.initialize()

    assert wrapper.move(width=0.0, force=1.0, clear_disabled=False)
    assert driver.reset_calls == 0
    assert driver.moves == [(0.0, 1.0)]
