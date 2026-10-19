from agx_arm_ctrl.command_watchdog import (
    ExternalCommandWatchdog,
    complete_command_values,
)


def test_complete_command_rejects_missing_duplicate_or_nonfinite_joints():
    required = ("joint1", "joint2", "gripper")
    assert complete_command_values(
        reversed(required), reversed((1.0, 2.0, 0.07)), required
    ) == {"joint1": 1.0, "joint2": 2.0, "gripper": 0.07}
    assert complete_command_values(required[:-1], (1.0, 2.0), required) is None
    assert complete_command_values(
        (*required, "unexpected"), (1.0, 2.0, 0.07, 9.0), required
    ) is None
    assert complete_command_values(
        ("joint1", "joint1", "gripper"), (1.0, 2.0, 0.07), required
    ) is None
    assert complete_command_values(required, (1.0, float("nan"), 0.07), required) is None


def test_external_command_watchdog_requires_fresh_commands_after_open():
    watchdog = ExternalCommandWatchdog(0.25)
    assert not watchdog.expired(10.0)
    watchdog.open(10.0)
    assert not watchdog.expired(10.25)
    assert watchdog.expired(10.251)
    watchdog.command(10.2)
    assert not watchdog.expired(10.4)
    assert watchdog.expired(10.1)
    watchdog.close()
    assert not watchdog.expired(100.0)
