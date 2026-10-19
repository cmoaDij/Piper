#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import Literal

from ....core.attritube_base import AttributeBase


class ArmMsgLeaderArmMoveToHome(AttributeBase):
    """
    transmit

    请求主导臂（Leader Arm）回零指令（基于 V1.7-4 版本后）

    CAN ID:
        0x191

    Args:
        mode: 请求回零模式
            0: 恢复 Leader-Follower 臂模式
            1: Leader Arm 回零
            2: Leader-Follower 臂一起回零
    """
    """
    transmit

    Request Leader Arm Move to Home Command (Based on version V1.7-4 and later)

    CAN ID:
        0x191

    Args:
        mode (int): Request return-to-zero mode.
            0: Restore leader-follower arm mode.
            1: Leader arm return-to-zero.
            2: Leader and follower arms return-to-zero together.
    """

    def __init__(self, mode: Literal[0, 1, 2] = 1):
        if mode not in [0, 1, 2]:
            raise ValueError(f"'mode' Value {mode} out of range [0, 1, 2]")
        self.mode = mode

