#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase
from typing import Union


class ArmMsgFeedbackIKJointStates(AttributeBase):
    '''
    feedback
    
    机械臂IK解算关节角度反馈,单位0.001度
    
    CAN ID: 
        0x2AA、0x2AB、0x2AC
    
    Args:
        joint_1: IK解算关节1角度
        joint_2: IK解算关节2角度
        joint_3: IK解算关节3角度
        joint_4: IK解算关节4角度
        joint_5: IK解算关节5角度
        joint_6: IK解算关节6角度
    '''
    '''
    feedback
    
    IK Computed Joint Angle Feedback for Robotic Arm, in 0.001 Degrees
    
    CAN ID: 
        0x2AA、0x2AB、0x2AC
    
    Args:
        joint_1: IK computed angle of joint 1, in radians.
        joint_2: IK computed angle of joint 2, in radians.
        joint_3: IK computed angle of joint 3, in radians.
        joint_4: IK computed angle of joint 4, in radians.
        joint_5: IK computed angle of joint 5, in radians.
        joint_6: IK computed angle of joint 6, in radians.
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


class ArmMsgFeedbackIKJointStates12(AttributeBase):
    '''CAN ID:
        0x2AA'''
    def __init__(self,
                 joint_1: Union[int, float] = 0,
                 joint_2: Union[int, float] = 0):
        self.joint_1 = joint_1
        self.joint_2 = joint_2


class ArmMsgFeedbackIKJointStates34(AttributeBase):
    '''CAN ID:
        0x2AB'''
    def __init__(self,
                 joint_3: Union[int, float] = 0,
                 joint_4: Union[int, float] = 0):
        self.joint_3 = joint_3
        self.joint_4 = joint_4


class ArmMsgFeedbackIKJointStates56(AttributeBase):
    '''CAN ID:
        0x2AC'''
    def __init__(self,
                 joint_5: Union[int, float] = 0,
                 joint_6: Union[int, float] = 0):
        self.joint_5 = joint_5
        self.joint_6 = joint_6
