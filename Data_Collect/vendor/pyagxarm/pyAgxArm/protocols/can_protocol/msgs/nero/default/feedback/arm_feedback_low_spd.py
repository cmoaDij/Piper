#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....piper.default import (
    ArmMsgFeedbackLowSpd as ArmMsgFeedbackLowSpdBase,
    ArmMsgFeedbackAllLowSpd as ArmMsgFeedbackAllLowSpdBase,
)
class ArmMsgFeedbackLowSpd(ArmMsgFeedbackLowSpdBase):
    '''
    feedback
    
    驱动器信息低速反馈 0x6

    节点 ID:
        0x1~0x07
    CAN ID:
        0X261~0x267

    Args:
        vol: 当前驱动器电压
        foc_temp: 驱动器温度
        motor_temp: 电机温度
        foc_status: 驱动器状态码
        bus_current: 当前驱动器电流,单位0.001A,1.5KG关节电机无母线电流采样,默认发送0
    
    位描述:
    
        Byte 0:电压高八位, uint16, 当前驱动器电压单位: 0.1V
        Byte 1:电压低八位
        Byte 2:驱动器温度高八位, int16, 单位: 1℃
        Byte 3:驱动器温度低八位
        Byte 4:电机温度,int8,单位: 1℃
        Byte 5:驱动器状态,uint8
            bit[0] 电源电压是否过低(0--正常; 1--过低)
            bit[1] 电机是否过温(0--正常; 1--过温)
            bit[2] 驱动器是否过流(0--正常; 1--过流)
            bit[3] 驱动器是否过温(0--正常; 1--过温)
            bit[4] 碰撞保护状态(0--正常; 1--触发保护)-7.25修改,之前为传感器状态
            bit[5] 驱动器错误状态(0: 正常; 1--错误)
            bit[6] 驱动器使能状态(1--使能; 0--失能)
            bit[7] 堵转保护状态(0--正常; 1--触发保护)-2024-7-25修改,之前为回零状态
        Byte 6:母线电流高八位,uint16,当前驱动器电流单位: 0.001A,1.5KG关节电机无母线电流采样,默认发送0
        Byte 7:母线电流低八位
    '''
    '''
    feedback
    
    Low-Speed Feedback of Drive Information 0x6

    Node ID:
        0x1~0x07

    CAN IDs:
        0x261~0x267

    Args:
        vol: Current driver voltage.
        foc_temp: Driver temperature.
        motor_temp: Motor temperature.
        foc_status: Driver status.
        bus_current: Current driver current.
    
    Bit Definitions:
    
        Byte 0: Bus Voltage (High Byte), uint16, unit: 0.1 V
        Byte 1: Bus Voltage (Low Byte)
        Byte 2: Drive Temperature (High Byte), int16, unit: 1°C
        Byte 3: Drive Temperature (Low Byte)
        Byte 4: Motor Temperature, int8, unit: 1°C
        Byte 5: Drive Status, uint8:
            bit[0]: Power voltage low (0: Normal, 1: Low)
            bit[1]: Motor over-temperature (0: Normal, 1: Over-temperature)
            bit[2]: Drive over-current (0: Normal, 1: Over-current)
            bit[3]: Drive over-temperature (0: Normal, 1: Over-temperature)
            bit[4]: Collision protection status (0: Normal, 1: Trigger protection) (Updated 7.25, previously sensor status)
            bit[5]: Drive error status (0: Normal, 1: Error)
            bit[6]: Drive enable status (1: Enabled, 0: Disabled)
            bit[7]: Stalling protection status (0: Normal, 1: Trigger protection) (Updated 7.25, previously zeroing status)
        Byte 6: Bus Current (High Byte), uint16, unit: 0.001 A, The 1.5KG joint motor has no bus current sampling and defaults to sending 0.
        Byte 7: Bus Current (Low Byte)
    '''
    pass

class ArmMsgFeedbackLowSpd7(ArmMsgFeedbackLowSpd):
    '''CAN ID:
        0x267'''
    pass

class ArmMsgFeedbackAllLowSpd(ArmMsgFeedbackAllLowSpdBase):
    '''CAN ID:
        0x261~0x267'''
    def __init__(self,
                 joint_1 = ArmMsgFeedbackLowSpd(),
                 joint_2 = ArmMsgFeedbackLowSpd(),
                 joint_3 = ArmMsgFeedbackLowSpd(),
                 joint_4 = ArmMsgFeedbackLowSpd(),
                 joint_5 = ArmMsgFeedbackLowSpd(),
                 joint_6 = ArmMsgFeedbackLowSpd(),
                 joint_7 = ArmMsgFeedbackLowSpd()):
        super().__init__(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
        self.joint_7 = joint_7
        