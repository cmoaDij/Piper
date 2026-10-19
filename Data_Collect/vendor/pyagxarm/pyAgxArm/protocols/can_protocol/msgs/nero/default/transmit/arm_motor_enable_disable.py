#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
from ....piper.default import ArmMsgMotorEnableDisableConfig as ArmMsgMotorEnableDisableConfigBase

class ArmMsgMotorEnableDisableConfig(ArmMsgMotorEnableDisableConfigBase):
    '''
    transmit
    
    电机使能/失能设置指令
    
    CAN ID:
        0x471
    
    Args:
        joint_index: 电机序号[1,8],8代表所有电机
        enable_flag: 使能标志位,0x01-失能;0x02-使能
    
    位描述:
    
        Byte 0: uint8, 关节电机序号。
                值域 1-8:1-7 代表关节驱动器序号,8代表夹爪电机,FF代表全部关节电机(包含夹爪)
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
        joint_index: Motor index [1, 8], where 8 represents all motors.
        enable_flag: Enable flag, 0x01 for disable, 0x02 for enable.

    Bit Description:

        Byte 0:
            motor_num, uint8, motor index.
            Range 1-8:
                1-7: Represents joint motor index.
                8: Represents gripper motor.
                0xFF: Represents all joint motors (including gripper).
        Byte 1:
            enable_flag, uint8, enable/disable.
            0x01: Disable.
            0x02: Enable.
    '''
    _VALID_JOINT_INDEX = [1, 2, 3, 4, 5, 6, 7, 8, 0xFF]

    def __init__(self, 
                 joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 8, 0xFF] = 0xFF,
                 enable_flag: Literal[0x01, 0x02] = 0x01):
        super().__init__(joint_index, enable_flag)
