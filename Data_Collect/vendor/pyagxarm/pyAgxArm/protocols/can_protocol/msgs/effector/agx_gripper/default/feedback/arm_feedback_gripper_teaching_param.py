#!/usr/bin/env python3
# -*-coding:utf8-*-
from .....core.attritube_base import AttributeBase
from typing import Optional, Union


class ArmMsgFeedbackGripperTeachingPendantParam(AttributeBase):
    """
    feedback

    夹爪/示教器参数反馈（V1.5-2 及之后版本）

    Gripper/Teaching Pendant Parameter Feedback (based on version V1.5-2 and later)

    CAN ID:
        0x47E

    Notes
    -----
    Payload layout (8 bytes):
    - Byte 0: teaching_range_per (uint8), percentage-style value
      - Range used by config API: [100, 200]
    - Byte 1: max_range_config (uint8)
      - Typical values: 0 / 70 / 100
    - Byte 2: teaching_friction (uint8)
      - Typical values: 1..10
    - Byte 3-7: reserved
    """

    def __init__(
        self,
        teaching_range_per: Optional[int] = None,
        max_range_config: Union[int, float, None] = None,
        teaching_friction: Optional[int] = None,
    ):
        self.teaching_range_per = teaching_range_per
        self.max_range_config = max_range_config
        self.teaching_friction = teaching_friction

    def clear(self):
        self.teaching_range_per = None
        self.max_range_config = None
        self.teaching_friction = None


__all__ = ["ArmMsgFeedbackGripperTeachingPendantParam"]
