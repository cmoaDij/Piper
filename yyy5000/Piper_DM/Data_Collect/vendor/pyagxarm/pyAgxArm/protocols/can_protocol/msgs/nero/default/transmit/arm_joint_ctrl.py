#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase
from ....piper.default import ArmMsgJointCtrl as ArmMsgJointCtrlBase
from typing import Union

class ArmMsgJointCtrl(ArmMsgJointCtrlBase):
    '''
    transmit
    
    机械臂关节控制,单位0.001度
    
    CAN ID:
        0x155,0x156,0x157,0x170
    
    Args:
        joint_1: joint_1角度
        joint_2: joint_2角度
        joint_3: joint_3角度
        joint_4: joint_4角度
        joint_5: joint_5角度
        joint_6: joint_6角度
        joint_7: joint_7角度
    '''
    '''
    transmit
    
    Robotic Arm Joint Control (Unit: 0.001°)

    CAN IDs:
        0x155, 0x156, 0x157, 0x170
    
    Args:
        joint_1: The target angle of joint 1 in 0.001°.
        joint_2: The target angle of joint 2 in 0.001°.
        joint_3: The target angle of joint 3 in 0.001°.
        joint_4: The target angle of joint 4 in 0.001°.
        joint_5: The target angle of joint 5 in 0.001°.
        joint_6: The target angle of joint 6 in 0.001°.
        joint_7: The target angle of joint 7 in 0.001°.
    '''
    def __init__(self, 
                 joint_1: Union[int, float] = 0,
                 joint_2: Union[int, float] = 0,
                 joint_3: Union[int, float] = 0,
                 joint_4: Union[int, float] = 0,
                 joint_5: Union[int, float] = 0,
                 joint_6: Union[int, float] = 0,
                 joint_7: Union[int, float] = 0):
        super().__init__(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
        self.joint_7 = joint_7

class ArmMsgJointCtrl7(AttributeBase):
    '''CAN ID:
        0x170'''
    def __init__(self, 
                 joint_7: int = 0):
        self.joint_7 = joint_7
