"""Piper controller-resident drag-teach command helpers."""

from enum import IntEnum
from typing import Any

from pyAgxArm.protocols.can_protocol.msgs.piper.default import (
    ArmMsgMotionCtrl,
)


class TeachCommand(IntEnum):
    """CAN 0x150 drag-teach command values from the Piper protocol."""

    START_RECORD = 0x01
    STOP_RECORD = 0x02
    REPLAY = 0x03
    PAUSE = 0x04
    RESUME = 0x05
    TERMINATE = 0x06
    MOVE_TO_START = 0x07


TEACH_STATUS_NAMES = {
    0x00: "off",
    0x01: "recording",
    0x02: "recording_stopped",
    0x03: "replaying",
    0x04: "paused",
    0x05: "resumed",
    0x06: "terminated",
    0x07: "moving_to_start",
}


def teach_status_name(value: int) -> str:
    """Return a stable diagnostic label for controller teach feedback."""
    status = int(value)
    return TEACH_STATUS_NAMES.get(status, f"unknown_0x{status:02x}")


def send_teach_command(arm: Any, command: TeachCommand) -> None:
    """Send one Piper drag-teach command through the existing CAN owner."""
    if not isinstance(command, TeachCommand):
        raise TypeError("command must be a TeachCommand")
    sender = getattr(arm, "_send_msg", None)
    if sender is None or not callable(sender):
        raise RuntimeError("the active pyAgxArm driver cannot send teach commands")
    sender(ArmMsgMotionCtrl(grag_teach_ctrl=int(command)))
