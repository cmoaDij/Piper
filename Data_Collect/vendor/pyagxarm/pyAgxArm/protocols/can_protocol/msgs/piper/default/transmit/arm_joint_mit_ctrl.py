#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase

class ArmMsgJointMitCtrl(AttributeBase):
    '''
    transmit
    
    机械臂关节mit控制
    
    CAN ID:
        0x15A,0x15B,0x15C,0x15D,0x15E,0x15F
    
    每个ID对应单个关节,因此有六个ID
    
    Args:
        p_des: 设定期望的目标位置
        v_des: 设定电机运动的速度
        kp: 比例增益，控制位置误差对输出力矩的影响
        kd: 微分增益，控制速度误差对输出力矩的影响
        t_ff: 目标力矩参考值，用于控制电机施加的力矩或扭矩
        crc: 循环冗余校验，用于数据完整性验证
    
    位描述:
    
        Byte 0: p_des [bit15~bit8] 高8位
        Byte 1: p_des [bit7~bit0]  低8位
        Byte 2: v_des [bit11~bit4] 低12位
        Byte 3: v_des [bit3~bit0], kp [bit11~bit8]
        Byte 4: Kp [bit7~bit0],      Kp给定参考值: 10
        Byte 5: kd [bit11~bit4]      低12位,kd给定参考值: 0.8
        Byte 6: kd [bit3~bit0] t_ff [bit7~bit4]
        Byte 7: t_ff [bit3~bit0] crc [bit3~bit0]
    '''
    '''
    transmit
    
    Robotic Arm Joint MIT Control

    CAN IDs:
        0x15A, 0x15B, 0x15C, 0x15D, 0x15E, 0x15F

    Each ID corresponds to a single joint, thus there are six IDs.
    
    Args:
        p_des: Desired target position
        v_des: Desired motor motion speed
        kp: Proportional gain, controls the influence of position error on output torque
        kd: Derivative gain, controls the influence of velocity error on output torque
        t_ff: Target torque reference value, used to control the motor's applied force or torque
        crc: Cyclic Redundancy Check for data integrity verification
    
    Bit Description:
    
        Byte 0	p_des	bit15~bit8	High 8 bits of p_des
        Byte 1	p_des	bit7~bit0	Low 8 bits of p_des
        Byte 2	v_des	bit11~bit4	Low 12 bits of v_des
        Byte 3	v_des, kp	bit3~bit0, bit11~bit8	Remaining 4 bits of v_des, high 4 bits of kp
        Byte 4	Kp	bit7~bit0	Low 8 bits of kp (default: 10)
        Byte 5	kd	bit11~bit4	Low 12 bits of kd (default: 0.8)
        Byte 6	kd, t_ff	bit3~bit0, bit7~bit4	Remaining 4 bits of kd, high 4 bits of t_ff
        Byte 7	t_ff, crc	bit3~bit0, bit3~bit0	Low 4 bits of t_ff, low 4 bits of crc
    '''
    def __init__(self, 
                 p_des: int = 0,
                 v_des: int = 0,
                 kp: int = 0,
                 kd: int = 0,
                 t_ff: int = 0,
                 crc: int = 0):
        self.p_des = p_des
        self.v_des = v_des
        self.kp = kp
        self.kd = kd
        self.t_ff = t_ff
        self.crc = crc

class ArmMsgJointMitCtrl1(ArmMsgJointMitCtrl):
    '''CAN ID:
        0x15A'''
    pass

class ArmMsgJointMitCtrl2(ArmMsgJointMitCtrl):
    '''CAN ID:
        0x15B'''
    pass

class ArmMsgJointMitCtrl3(ArmMsgJointMitCtrl):
    '''CAN ID:
        0x15C'''
    pass

class ArmMsgJointMitCtrl4(ArmMsgJointMitCtrl):
    '''CAN ID:
        0x15D'''
    pass

class ArmMsgJointMitCtrl5(ArmMsgJointMitCtrl):
    '''CAN ID:
        0x15E'''
    pass

class ArmMsgJointMitCtrl6(ArmMsgJointMitCtrl):
    '''CAN ID:
        0x15F'''
    pass
