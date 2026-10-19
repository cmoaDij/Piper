#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase
from typing import Union

class ArmMsgFeedbackEndPose(AttributeBase):
    '''
    feedback
    
    机械臂末端姿态反馈,单位0.001mm
    
    CAN ID: 
        0x2A2、0x2A3、0x2A4
    
    Args:
        X_axis: X坐标
        Y_axis: Y坐标
        Z_axis: Z坐标
        RX_axis: RX角度
        RY_axis: RY角度
        RZ_axis: RZ角度
    '''
    '''
    feedback
    
    End-Effector Pose Feedback for the Robotic Arm, unit: 0.001 mm.
    
    CAN ID: 
        0x2A2、0x2A3、0x2A4
    
    Args:
        X_axis: X-coordinate.
        Y_axis: Y-coordinate.
        Z_axis: Z-coordinate.
        RX_axis: Rotation angle around the X-axis (RX).
        RY_axis: Rotation angle around the Y-axis (RY).
        RZ_axis: Rotation angle around the Z-axis (RZ).
    '''
    def __init__(self, 
                 X_axis: Union[int, float] = 0,
                 Y_axis: Union[int, float] = 0,
                 Z_axis: Union[int, float] = 0,
                 RX_axis: Union[int, float] = 0,
                 RY_axis: Union[int, float] = 0,
                 RZ_axis: Union[int, float] = 0):
        self.X_axis = X_axis
        self.Y_axis = Y_axis
        self.Z_axis = Z_axis
        self.RX_axis = RX_axis
        self.RY_axis = RY_axis
        self.RZ_axis = RZ_axis

class ArmMsgFeedbackEndPoseXY(AttributeBase):
    '''CAN ID:
        0x2A2'''
    def __init__(self, 
                 X_axis: Union[int, float] = 0,
                 Y_axis: Union[int, float] = 0):
        self.X_axis = X_axis
        self.Y_axis = Y_axis

class ArmMsgFeedbackEndPoseZRX(AttributeBase):
    '''CAN ID:
        0x2A3'''
    def __init__(self, 
                 Z_axis: Union[int, float] = 0,
                 RX_axis: Union[int, float] = 0):
        self.Z_axis = Z_axis
        self.RX_axis = RX_axis

class ArmMsgFeedbackEndPoseRYRZ(AttributeBase):
    '''CAN ID:
        0x2A4'''
    def __init__(self, 
                 RY_axis: Union[int, float] = 0,
                 RZ_axis: Union[int, float] = 0):
        self.RY_axis = RY_axis
        self.RZ_axis = RZ_axis
