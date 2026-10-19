#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
from ....piper.default import ArmMsgJointConfig as ArmMsgJointConfigBase

class ArmMsgJointConfig(ArmMsgJointConfigBase):
    '''
    transmit
    
    关节设置指令

    CAN ID:
        0x475

    Args:
        joint_index: 关节电机序号,[1, 8]
        set_motor_current_pos_as_zero: 设置当前位置为零点, 有效值-0xAE
        acc_param_config_is_effective_or_not: 加速度参数设置是否生效, 有效值-0xAE
        max_joint_acc: 最大关节加速度,单位0.01rad/s^2(0x7FFF为设定无效数值),输入范围[0, 500]-->[0 rad/s^2, 5.0 rad/s^2]
        clear_joint_err: 清除关节错误代码, 有效值-0xAE
    
    位描述:
    
        Byte 0: 关节电机序号 uint8, 值域 1-8
                1-7 代表关节驱动器序号；
                8 代表全部关节电机
        Byte 1: 设置N号电机当前位置为零点: uint8, 有效值-0xAE
        Byte 2: 加速度参数设置是否生效: uint8, 有效值-0xAE
        Byte 3: 最大关节加速度 H: uint16, 单位 0.01rad/s^2.(基于V1.5-2版本后增加无效数值0x7FFF)
        Byte 4: 最大关节加速度 L
        Byte 5: 清除关节错误代码: uint8, 有效值-0xAE
        Byte 6: 保留
        Byte 7: 保留
    '''
    '''
    transmit
    
    Joint Configuration Command

    CAN ID:
    0x475
    
    Args:
        joint_index: Joint motor number.
            Value range: 1-7 represents individual joint motor numbers.
            Value 8 applies to all joint motors.
        set_motor_current_pos_as_zero: Command to set the current position of the specified joint motor as zero, with a valid value of 0xAE.
        acc_param_config_is_effective_or_not: Indicates whether the acceleration parameter configuration is effective, with a valid value of 0xAE.
        max_joint_acc: Maximum joint acceleration, unit: 0.01rad/s^2, 0x7FFF is defined as the invalid value.Range is [0, 500]-->[0 rad/s^2, 5.0 rad/s^2]
        clear_joint_err: Command to clear joint error codes, with a valid value of 0xAE.

    Bit Description:

        Byte 0: Joint motor number (uint8).
                - 1-7: Corresponds to individual joint motor numbers.
                - 8: Represents all joint motors.
        Byte 1: Set the current position of the specified joint motor as zero (uint8).
                - Valid value: 0xAE.
        Byte 2: Determines if the acceleration parameter configuration is effective (uint8).
                - Valid value: 0xAE.
        Byte 3-4: Maximum joint acceleration (uint16).(Based on version V1.5-2 and later, the invalid value 0x7FFF is added.)
                - Unit: 0.01rad/s^2.
                - Byte 3: High byte, Byte 4: Low byte.
        Byte 5: Clear joint error code (uint8).
                - Valid value: 0xAE.
        Byte 6: Reserved
        Byte 7: Reserved
    '''
    _VALID_JOINT_INDEX = [1, 2, 3, 4, 5, 6, 7, 8, 0xFF]

    def __init__(self, 
                 joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 8, 0xFF] = 0xFF, 
                 set_motor_current_pos_as_zero: Literal[0x00, 0xAE] = 0, 
                 acc_param_config_is_effective_or_not: Literal[0x00, 0xAE] = 0,
                 max_joint_acc: int = 500,
                 clear_joint_err: Literal[0x00, 0xAE] = 0):
        super().__init__(joint_index, set_motor_current_pos_as_zero, acc_param_config_is_effective_or_not, max_joint_acc, clear_joint_err)
