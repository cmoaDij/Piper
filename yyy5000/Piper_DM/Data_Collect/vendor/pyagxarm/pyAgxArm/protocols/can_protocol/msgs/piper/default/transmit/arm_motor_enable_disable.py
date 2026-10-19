#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
from ....core.attritube_base import AttributeBase

class ArmMsgMotorEnableDisableConfig(AttributeBase):
    '''
    transmit
    
    电机使能/失能设置指令
    
    CAN ID:
        0x471
    
    Args:
        joint_index: 电机序号[1,7],7代表所有电机
        enable_flag: 使能标志位,0x01-失能;0x02-使能
    
    位描述:
    
        Byte 0: uint8, 关节电机序号。
                值域 1-7:1-6 代表关节驱动器序号,7代表夹爪电机,FF代表全部关节电机(包含夹爪)
        Byte 1: uint8, 使能/失能。
                0x01 : 失能
                0x02 : 使能
    '''
    '''
    transmit
    
    Motor Enable/Disable Command

    CAN ID:
        0x471

    Args:
        joint_index: Motor index [1, 7], where 7 represents all motors.
        enable_flag: Enable flag, 0x01 for disable, 0x02 for enable.

    Bit Description:

        Byte 0:
            joint_index, uint8, motor index.
            Range 1-7:
                1-6: Represents joint motor index.
                7: Represents gripper motor.
                0xFF: Represents all joint motors (including gripper).
        Byte 1:
            enable_flag, uint8, enable/disable.
            0x01: Disable.
            0x02: Enable.
    '''
    _VALID_JOINT_INDEX = [1, 2, 3, 4, 5, 6, 7, 0xFF]

    def __init__(self, 
                 joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 0xFF] = 0xFF,
                 enable_flag: Literal[0x01, 0x02] = 0x01):
        if joint_index not in self._VALID_JOINT_INDEX:
            raise ValueError(f"'joint_index' Value {joint_index} out of range {self._VALID_JOINT_INDEX}")
        if enable_flag not in [0x01, 0x02]:
            raise ValueError(f"'enable_flag' Value {enable_flag} out of range [0x01, 0x02]")
        if joint_index == 0xFF:
            joint_index = self._VALID_JOINT_INDEX[-2]
        self.joint_index = joint_index
        self.enable_flag = enable_flag
