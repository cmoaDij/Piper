from enum import unique

from ....core import IntEnumBase
from ...default.transmit.arm_mode_ctrl import ArmMsgModeCtrl


class ArmMsgModeCtrlV188(ArmMsgModeCtrl):
    """v188 mode control: MotionMode.MIT = 0x06."""

    _VALID_MOVE_MODE = [0x00, 0x01, 0x02, 0x03, 0x06, 0x05]

    class Enums(ArmMsgModeCtrl.Enums):
        @unique
        class MotionMode(IntEnumBase):
            P = 0x00
            J = 0x01
            L = 0x02
            C = 0x03
            MIT = 0x06
            CPV = 0x05
            UNKNOWN = 0xFF
