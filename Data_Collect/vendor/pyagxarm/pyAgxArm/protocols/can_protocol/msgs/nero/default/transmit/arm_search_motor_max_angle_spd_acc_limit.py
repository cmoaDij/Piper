#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
from ....piper.default import ArmMsgSearchMotorMaxAngleSpdAccLimit as ArmMsgSearchMotorMaxAngleSpdAccLimitBase

class ArmMsgSearchMotorMaxAngleSpdAccLimit(ArmMsgSearchMotorMaxAngleSpdAccLimitBase):
    '''
    transmit
    
    查询电机角度/最大速度/最大加速度限制指令

    CAN ID:
        0x472

    Args:
        joint_index: 关节电机序号,1-7
        search_content: 查询内容,0x01-查询电机角度/最大速度,0x02-查询电机最大加速度限制

    位描述:
    
        :Byte 0 joint_index: uint8, 关节电机序号。
                值域 1-7,1-7 代表关节驱动器序号
        :Byte 1 search_content: uint8, 查询内容。
                0x01 : 查询电机角度/最大速度
                0x02 : 查询电机最大加速度限制
    '''
    '''
    transmit
    
    Motor Angle/Max Speed/Max Acceleration Limit Query Command

    CAN ID:
        0x472

    Args:
        joint_index: Motor joint number.
        search_content: Query content.

    Bit Description:

        Byte 0: uint8, motor joint number.
            Value range: 1-7.
                1-7: Represent joint driver numbers.
        Byte 1: uint8, query content.
            0x01: Query motor angle/max speed.
            0x02: Query motor max acceleration limit.
    '''
    _VALID_JOINT_INDEX = [1, 2, 3, 4, 5, 6, 7]

    def __init__(self, 
                 joint_index: Literal[1, 2, 3, 4, 5, 6, 7] = 1,
                 search_content: Literal[0x01, 0x02] = 0x01):
        super().__init__(joint_index, search_content)
