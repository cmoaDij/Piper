#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase

class ArmMsgEndPoseCtrl(AttributeBase):
    '''
    transmit
    
    机械臂运动控制直角坐标系指令
    
    CAN ID:
        0x152,0x153,0x154

    Args:
        X_axis: X坐标,单位0.001mm
        Y_axis: Y坐标,单位0.001mm
        Z_axis: Z坐标,单位0.001mm
        RX_axis: RX角度,单位0.001度
        RY_axis: RY角度,单位0.001度
        RZ_axis: RZ角度,单位0.001度
    '''
    '''
    transmit
    
    Robotic Arm Motion Control Command in Cartesian Coordinate System

    CAN ID:
        0x152, 0x153, 0x154

    Args:
        X_axis: X-axis coordinate, in 0.001 mm.
        Y_axis: Y-axis coordinate, in 0.001 mm.
        Z_axis: Z-axis coordinate, in 0.001 mm.
        RX_axis: Rotation about X-axis, in 0.001 degrees.
        RY_axis: Rotation about Y-axis, in 0.001 degrees.
        RZ_axis: Rotation about Z-axis, in 0.001 degrees.
    '''
    pass

class ArmMsgEndPoseCtrlXY(AttributeBase):
    '''CAN ID:
        0x152'''
    def __init__(self, 
                 X_axis: int = 0,
                 Y_axis: int = 0
                 ):
        self.X_axis = X_axis
        self.Y_axis = Y_axis

class ArmMsgEndPoseCtrlZRX(AttributeBase):
    '''CAN ID:
        0x153'''
    def __init__(self, 
                 Z_axis: int = 0,
                 RX_axis: int = 0
                 ):
        self.Z_axis = Z_axis
        self.RX_axis = RX_axis

class ArmMsgEndPoseCtrlRYRZ(AttributeBase):
    '''CAN ID:
        0x154'''
    def __init__(self, 
                 RY_axis: int = 0,
                 RZ_axis: int = 0
                 ):
        self.RY_axis = RY_axis
        self.RZ_axis = RZ_axis
