#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import Union
from typing_extensions import Literal
from ....piper.default import (
    ArmMsgFeedbackCurrentMotorMaxAccLimit as ArmMsgFeedbackCurrentMotorMaxAccLimitBase,
    ArmMsgFeedbackAllCurrentMotorMaxAccLimit as ArmMsgFeedbackAllCurrentMotorMaxAccLimitBase,
)

class ArmMsgFeedbackCurrentMotorMaxAccLimit(ArmMsgFeedbackCurrentMotorMaxAccLimitBase):
    def __init__(self,
                 joint_index: Literal[0, 1, 2, 3, 4, 5, 6, 7] = 0,
                 max_joint_acc: Union[int, float, None] = None):
        super().__init__(joint_index, max_joint_acc)

class ArmMsgFeedbackAllCurrentMotorMaxAccLimit(ArmMsgFeedbackAllCurrentMotorMaxAccLimitBase):
    '''
    feedback
    
    反馈当前电机最大加速度限制

    CAN ID:
        0x47C

    Args:
        joint_index: 关节电机序号
        max_joint_acc: 最大关节加速度
    
    位描述:

        Byte 0: 关节序号, uint8, 值域 1-6(1-6 代表关节驱动器序号)
        Byte 1: 最大关节加速度 H, uint16, 单位 0.001rad/^2
        Byte 2: 最大关节加速度 L
    '''
    '''
    feedback
    
    Feedback on Current Motor Maximum Acceleration Limit

    CAN ID: 
        0x47C

    Args:
        joint_index: Joint motor number.
        max_joint_acc: Maximum joint acceleration.
    
    Bit Description:

        Byte 0: Joint Index, uint8, range 1-6(1-6 represent the joint motor index)
        Byte 1: Maximum Joint Acceleration (High Byte), uint16, unit: 0.001rad/^2
        Byte 2: Maximum Joint Acceleration (Low Byte)
    '''
    _VALID_JOINT_INDEX = [0, 1, 2, 3, 4, 5, 6, 7]

    def __init__(self, 
                 joint_index: Literal[0, 1, 2, 3, 4, 5, 6, 7] = 0):
        super().__init__(joint_index)
