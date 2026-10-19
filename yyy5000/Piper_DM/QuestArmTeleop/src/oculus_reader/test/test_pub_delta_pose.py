"""Unit tests for Quest teleoperation command mappings."""

from pathlib import Path
import sys


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from gripper_mapping import gripper_position_from_trigger  # noqa: E402
from button_edge import RisingEdge  # noqa: E402


def test_released_trigger_opens_gripper():
    assert gripper_position_from_trigger(0.0, 0.07) == 0.07


def test_pressed_trigger_closes_gripper():
    assert gripper_position_from_trigger(1.0, 0.07) == 0.0


def test_trigger_is_clamped_to_valid_range():
    assert gripper_position_from_trigger(-1.0, 0.07) == 0.07
    assert gripper_position_from_trigger(2.0, 0.07) == 0.0


def test_invalid_trigger_fails_open():
    assert gripper_position_from_trigger("invalid", 0.07) == 0.07
    assert gripper_position_from_trigger(float("nan"), 0.07) == 0.07


def test_reset_button_fires_once_until_released():
    edge = RisingEdge()

    assert edge.update(False) is False
    assert edge.update(True) is True
    assert edge.update(True) is False
    assert edge.update(False) is False
    assert edge.update(True) is True
