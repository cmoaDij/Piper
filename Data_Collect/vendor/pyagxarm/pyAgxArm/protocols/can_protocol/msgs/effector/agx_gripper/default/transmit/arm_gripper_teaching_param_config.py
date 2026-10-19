#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import Literal

from .....core.attritube_base import AttributeBase


class ArmMsgGripperTeachingPendantParamConfig(AttributeBase):
    """
    transmit

    夹爪/示教器参数配置指令

    Gripper/Teaching Pendant Parameter Configuration Command

    CAN ID:
        0x47D

    Notes
    -----
    Payload layout (8 bytes):
    - Byte 0: teaching_range_per (uint8), range: [100, 200]
    - Byte 1: max_range_config (uint8), allowed: 0 / 70 / 100
    - Byte 2: teaching_friction (uint8), allowed: 1..10
    - Byte 3-7: reserved (should be 0)

    This command is usually applied together with querying feedback `0x47E`.
    """

    def __init__(
        self,
        teaching_range_per: int = 100,
        max_range_config: Literal[0, 70, 100] = 0,
        teaching_friction: Literal[1, 2, 3, 4, 5, 6, 7, 8, 9, 10] = 1,
    ):
        if not (100 <= teaching_range_per <= 200):
            raise ValueError(
                f"'teaching_range_per' Value {teaching_range_per} out of range "
                "[100, 200]"
            )
        if max_range_config not in [0, 70, 100]:
            raise ValueError(
                f"'max_range_config' Value {max_range_config} out of range [0,70,100]"
            )
        if teaching_friction not in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
            raise ValueError(
                f"'teaching_friction' Value {teaching_friction} out of range "
                "[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]"
            )
        self.teaching_range_per = teaching_range_per
        self.max_range_config = max_range_config
        self.teaching_friction = teaching_friction


__all__ = ["ArmMsgGripperTeachingPendantParamConfig"]
