#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase
from typing import Union

class ArmMsgJointCtrl(AttributeBase):
    '''
    transmit
    
    机械臂关节控制,单位0.001度
    
    CAN ID:
        0x155,0x156,0x157
    
    Args:
        joint_1: joint_1角度
        joint_2: joint_2角度
        joint_3: joint_3角度
        joint_4: joint_4角度
        joint_5: joint_5角度
        joint_6: joint_6角度
    '''
    '''
    transmit
    
    Robotic Arm Joint Control (Unit: 0.001°)

    CAN IDs:
        0x155, 0x156, 0x157
    
    Args:
        joint_1: The target angle of joint 1 in 0.001°.
        joint_2: The target angle of joint 2 in 0.001°.
        joint_3: The target angle of joint 3 in 0.001°.
        joint_4: The target angle of joint 4 in 0.001°.
        joint_5: The target angle of joint 5 in 0.001°.
        joint_6: The target angle of joint 6 in 0.001°.
    '''
    def __init__(self, 
                 joint_1: Union[int, float] = 0,
                 joint_2: Union[int, float] = 0,
                 joint_3: Union[int, float] = 0,
                 joint_4: Union[int, float] = 0,
                 joint_5: Union[int, float] = 0,
                 joint_6: Union[int, float] = 0):
        self.joint_1 = joint_1
        self.joint_2 = joint_2
        self.joint_3 = joint_3
        self.joint_4 = joint_4
        self.joint_5 = joint_5
        self.joint_6 = joint_6

class ArmMsgJointCtrl12(AttributeBase):
    '''CAN ID:
        0x155'''
    def __init__(self, 
                 joint_1: int = 0,
                 joint_2: int = 0):
        self.joint_1 = joint_1
        self.joint_2 = joint_2

class ArmMsgJointCtrl34(AttributeBase):
    '''CAN ID:
        0x156'''
    def __init__(self, 
                 joint_3: int = 0,
                 joint_4: int = 0):
        self.joint_3 = joint_3
        self.joint_4 = joint_4

class ArmMsgJointCtrl56(AttributeBase):
    '''CAN ID:
        0x157'''
    def __init__(self, 
                 joint_5: int = 0,
                 joint_6: int = 0):
        self.joint_5 = joint_5
        self.joint_6 = joint_6
        