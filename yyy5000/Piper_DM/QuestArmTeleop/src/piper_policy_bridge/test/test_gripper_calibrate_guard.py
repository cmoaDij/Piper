"""Tests for the explicit gripper-calibration CLI contract."""

from piper_policy_bridge.gripper_calibrate_guard import build_parser


def test_physical_closure_confirmation_is_explicit():
    parser = build_parser()
    arguments = parser.parse_args(["--confirm-physically-closed"])
    assert arguments.confirm_physically_closed
    assert arguments.service == "/calibrate_gripper_closed"
    assert arguments.timeout == 10.0
