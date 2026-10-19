#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....piper.default import (
    ArmMsgJointMitCtrl as ArmMsgJointMitCtrlBase,
)

class ArmMsgJointMitCtrl(ArmMsgJointMitCtrlBase):
    '''
    transmit
    
    机械臂关节mit控制
    
    CAN ID:
        0x15A,0x15B,0x15C,0x15D,0x15E,0x15F,0x160
    
    每个ID对应单个关节,因此有七个ID
    
    Args:
        pos_ref: 设定期望的目标位置
        vel_ref: 设定电机运动的速度
        kp: 比例增益，控制位置误差对输出力矩的影响
        kd: 微分增益，控制速度误差对输出力矩的影响
        t_ref: 目标力矩参考值，用于控制电机施加的力矩或扭矩
        crc: 循环冗余校验，用于数据完整性验证
    
    位描述:
    
        Byte 0: Pos_ref [bit15~bit8] 高8位
        Byte 1: Pos_ref [bit7~bit0]  低8位
        Byte 2: Vel_ref [bit11~bit4] 低12位
        Byte 3: Vel_ref [bit3~bit0], Kp [bit11~bit8]
        Byte 4: Kp [bit7~bit0],      Kp给定参考值: 10
        Byte 5: Kd [bit11~bit4]      低12位,Kd给定参考值: 0.8
        Byte 6: Kd [bit3~bit0] T_ref [bit7~bit4]
        Byte 7: T_ref [bit3~bit0] CRC [bit3~bit0]
    '''
    '''
    transmit
    
    Robotic Arm Joint MIT Control

    CAN IDs:
        0x15A, 0x15B, 0x15C, 0x15D, 0x15E, 0x15F, 0x160

    Each ID corresponds to a single joint, thus there are seven IDs.
    
    Args:
        pos_ref: Desired target position
        vel_ref: Desired motor motion speed
        kp: Proportional gain, controls the influence of position error on output torque
        kd: Derivative gain, controls the influence of velocity error on output torque
        t_ref: Target torque reference value, used to control the motor's applied force or torque
        crc: Cyclic Redundancy Check for data integrity verification
    
    Bit Description:
    
        Byte 0	Pos_ref	bit15~bit8	High 8 bits of pos_ref
        Byte 1	Pos_ref	bit7~bit0	Low 8 bits of pos_ref
        Byte 2	Vel_ref	bit11~bit4	Low 12 bits of vel_ref
        Byte 3	Vel_ref, Kp	bit3~bit0, bit11~bit8	Remaining 4 bits of vel_ref, high 4 bits of kp
        Byte 4	Kp	bit7~bit0	Low 8 bits of kp (default: 10)
        Byte 5	Kd	bit11~bit4	Low 12 bits of kd (default: 0.8)
        Byte 6	Kd, T_ref	bit3~bit0, bit7~bit4	Remaining 4 bits of kd, high 4 bits of t_ref
        Byte 7	T_ref, CRC	bit3~bit0, bit3~bit0	Low 4 bits of t_ref, low 4 bits of crc
    '''
    pass

class ArmMsgJointMitCtrl7(ArmMsgJointMitCtrl):
    '''CAN ID:
        0x160'''
    pass
