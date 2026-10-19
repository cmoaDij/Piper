from enum import unique

from ....core import IntEnumBase
from ...default.feedback.arm_feedback_status import (
    ArmMsgFeedbackStatusEnum,
    ArmMsgFeedbackStatus,
)


class ArmMsgFeedbackStatusEnumV111(ArmMsgFeedbackStatusEnum):
    """v111 feedback status enums: ModeFeedback.MOVE_MIT = 0x06."""

    @unique
    class ModeFeedback(IntEnumBase):
        MOVE_P = 0x00
        MOVE_J = 0x01
        MOVE_L = 0x02
        MOVE_C = 0x03
        MOVE_MIT = 0x06
        MOVE_CPV = 0x05
        UNKNOWN = 0xFF


class ArmMsgFeedbackStatusV111(ArmMsgFeedbackStatus):
    """v111 feedback status: uses ArmMsgFeedbackStatusEnumV111 for mode_feedback."""

    @property
    def mode_feedback(self) -> ArmMsgFeedbackStatusEnumV111.ModeFeedback:
        return self._mode_feedback

    @mode_feedback.setter
    def mode_feedback(self, value: int):
        if isinstance(value, ArmMsgFeedbackStatusEnumV111.ModeFeedback):
            self._mode_feedback = value
        else:
            self._mode_feedback = ArmMsgFeedbackStatusEnumV111.ModeFeedback.match_value(value)
