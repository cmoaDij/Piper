#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase
from ....piper.default import ArmMsgFeedbackJointStates as ArmMsgFeedbackJointStatesBase
from typing import Union

class ArmMsgFeedbackJointStates(ArmMsgFeedbackJointStatesBase):
    '''
    feedback
    
    机械臂关节角度反馈,单位0.001度
    
    CAN ID: 
        0x2A5、0x2A6、0x2A7、0x2A9
    
    Args:
        joint_1: 关节1反馈角度
        joint_2: 关节2反馈角度
        joint_3: 关节3反馈角度
        joint_4: 关节4反馈角度
        joint_5: 关节5反馈角度
        joint_6: 关节6反馈角度
        joint_7: 关节7反馈角度
    '''
    '''
    feedback
    
    Joint Angle Feedback for Robotic Arm, in 0.001 Degrees
    
    CAN ID: 
        0x2A5、0x2A6、0x2A7、0x2A9
    
    Args:
        joint_1: Feedback angle of joint 1, in 0.001 degrees.
        joint_2: Feedback angle of joint 2, in 0.001 degrees.
        joint_3: Feedback angle of joint 3, in 0.001 degrees.
        joint_4: Feedback angle of joint 4, in 0.001 degrees.
        joint_5: Feedback angle of joint 5, in 0.001 degrees.
        joint_6: Feedback angle of joint 6, in 0.001 degrees.
        joint_7: Feedback angle of joint 7, in 0.001 degrees.
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

class ArmMsgFeedbackJointStates7(AttributeBase):
    '''CAN ID:
        0x2A9'''
    def __init__(self, 
                 joint_7: int = 0):
        self.joint_7 = joint_7
        