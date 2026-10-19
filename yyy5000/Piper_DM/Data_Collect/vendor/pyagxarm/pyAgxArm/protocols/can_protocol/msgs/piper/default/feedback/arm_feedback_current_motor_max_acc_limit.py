#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import Union, List
from typing_extensions import Literal
from ....core.attritube_base import AttributeBase

class ArmMsgFeedbackCurrentMotorMaxAccLimit(AttributeBase):
    def __init__(self,
                 joint_index: Literal[0, 1, 2, 3, 4, 5, 6] = 0,
                 max_joint_acc: Union[int, float, None] = None):
        self.joint_index = joint_index
        self.max_joint_acc = max_joint_acc
    
    def clear(self):
        self.max_joint_acc = None

class ArmMsgFeedbackAllCurrentMotorMaxAccLimit(AttributeBase):
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
    _VALID_JOINT_INDEX = [0, 1, 2, 3, 4, 5, 6]

    def __init__(self, 
                 joint_index: Literal[0, 1, 2, 3, 4, 5, 6] = 0):
        if joint_index not in self._VALID_JOINT_INDEX:
            raise ValueError(f"'joint_index' Value {joint_index} out of range {self._VALID_JOINT_INDEX}")
        self.joints: List[ArmMsgFeedbackCurrentMotorMaxAccLimit] = [ArmMsgFeedbackCurrentMotorMaxAccLimit(joint_index=i) for i in self._VALID_JOINT_INDEX[1:]]
        self.joint_index = joint_index
        