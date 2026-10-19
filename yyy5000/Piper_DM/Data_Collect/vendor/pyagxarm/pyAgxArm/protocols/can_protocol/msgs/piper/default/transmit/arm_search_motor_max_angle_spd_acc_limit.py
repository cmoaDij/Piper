#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
from ....core.attritube_base import AttributeBase

class ArmMsgSearchMotorMaxAngleSpdAccLimit(AttributeBase):
    '''
    transmit
    
    查询电机角度/最大速度/最大加速度限制指令

    CAN ID:
        0x472

    Args:
        joint_index: 关节电机序号,1-6
        search_content: 查询内容,0x01-查询电机角度/最大速度,0x02-查询电机最大加速度限制

    位描述:
    
        :Byte 0 joint_index: uint8, 关节电机序号。
                值域 1-6,1-6 代表关节驱动器序号
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
            Value range: 1-6.
                1-6: Represent joint driver numbers.
        Byte 1: uint8, query content.
            0x01: Query motor angle/max speed.
            0x02: Query motor max acceleration limit.
    '''
    _VALID_JOINT_INDEX = [1, 2, 3, 4, 5, 6]

    def __init__(self, 
                 joint_index: Literal[1, 2, 3, 4, 5, 6] = 1,
                 search_content: Literal[0x01, 0x02] = 0x01):
        if joint_index not in self._VALID_JOINT_INDEX:
            raise ValueError(f"'joint_index' Value {joint_index} out of range {self._VALID_JOINT_INDEX}")
        if search_content not in [0x01, 0x02]:
            raise ValueError(f"'search_content' Value {search_content} out of range [0x01, 0x02]")
        self.joint_index = joint_index
        self.search_content = search_content
