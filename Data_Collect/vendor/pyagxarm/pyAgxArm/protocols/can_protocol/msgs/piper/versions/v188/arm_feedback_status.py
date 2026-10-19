from enum import unique

from ....core import IntEnumBase
from ...default.feedback.arm_feedback_status import (
    ArmMsgFeedbackStatusEnum,
    ArmMsgFeedbackStatus,
)


class ArmMsgFeedbackStatusEnumV188(ArmMsgFeedbackStatusEnum):
    """v188 feedback status enums: ModeFeedback.MOVE_MIT = 0x06."""

    @unique
    class ModeFeedback(IntEnumBase):
        MOVE_P = 0x00
        MOVE_J = 0x01
        MOVE_L = 0x02
        MOVE_C = 0x03
        MOVE_MIT = 0x06
        MOVE_CPV = 0x05
        UNKNOWN = 0xFF


class ArmMsgFeedbackStatusV188(ArmMsgFeedbackStatus):
    """v188 feedback status: uses ArmMsgFeedbackStatusEnumV188 for mode_feedback."""

    @property
    def mode_feedback(self) -> ArmMsgFeedbackStatusEnumV188.ModeFeedback:
        return self._mode_feedback

    @mode_feedback.setter
    def mode_feedback(self, value: int):
        if isinstance(value, ArmMsgFeedbackStatusEnumV188.ModeFeedback):
            self._mode_feedback = value
        else:
            self._mode_feedback = ArmMsgFeedbackStatusEnumV188.ModeFeedback.match_value(value)
