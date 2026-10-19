#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import Union
from typing_extensions import Literal
from ....piper.default import (
    ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd as ArmMsgFeedbackCurrentMotorAngleLimitMaxSpdBase,
    ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd as ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpdBase,
)

class ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd(ArmMsgFeedbackCurrentMotorAngleLimitMaxSpdBase):
    def __init__(self, joint_index: Literal[0, 1, 2, 3, 4, 5, 6, 7] = 0,
                 max_angle_limit: Union[int, float, None] = None,
                 min_angle_limit: Union[int, float, None] = None,
                 max_joint_spd: Union[int, float, None] = None):
        super().__init__(joint_index, max_angle_limit, min_angle_limit, max_joint_spd)

class ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd(ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpdBase):
    '''
    feedback
    
    反馈当前电机限制角度/最大速度
    
    CAN ID:
        0x473
    
    Args:
        joint_index: 关节电机序号
        max_angle_limit: 最大角度限制
        min_angle_limit: 最小角度限制
        max_joint_spd: 最大关节速度

    位描述:

        Byte 0: 关节电机序号, uint8
        Byte 1: 最大角度限制H,uint16, 单位 0.1度
        Byte 2: 最大角度限制L
        Byte 3: 最小角度限制H, uint16, 单位 0.1度
        Byte 4: 最小角度限制L
        Byte 5: 最大关节速度H, uint16, 单位 0.001rad/s
        Byte 6: 最大关节速度L
        Byte 7: 保留
    '''
    '''
    feedback
    
    Feedback on Current Motor Angle Limits/Maximum Speed

    CAN ID:
        0x473

    Args:
        joint_index: Joint motor number.
        max_angle_limit: Maximum angle limit.
        min_angle_limit: Minimum angle limit.
        max_joint_spd: Maximum joint speed.
    
    Bit Description:

        Byte 0: Joint Motor Index, uint8
        Byte 1: Maximum Angle Limit (High Byte), uint16, unit: 0.1°
        Byte 2: Maximum Angle Limit (Low Byte)
        Byte 3: Minimum Angle Limit (High Byte), uint16, unit: 0.1°
        Byte 4: Minimum Angle Limit (Low Byte)
        Byte 5: Maximum Joint Speed (High Byte), uint16, unit: 0.001 rad/s
        Byte 6: Maximum Joint Speed (Low Byte)
        Byte 7: Reserved
    '''
    _VALID_JOINT_INDEX = [0, 1, 2, 3, 4, 5, 6, 7]

    def __init__(self, 
                 joint_index: Literal[0, 1, 2, 3, 4, 5, 6, 7] = 0):
        super().__init__(joint_index)
