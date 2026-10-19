#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
from enum import unique
from ....core import IntEnumBase
from ....piper.default import ArmMsgModeCtrl as ArmMsgModeCtrlBase

class ArmMsgModeCtrl(ArmMsgModeCtrlBase):
    '''
    transmit
    
    机械臂模式控制指令

    CAN ID:
        0x151
    
    Args:
        ctrl_mode: 控制模式
        move_mode: MOVE模式
        move_spd_rate_ctrl: 运动速度百分比
        mit_mode: mit模式
        residence_time: 离线轨迹点停留时间
        installation_pos: 安装位置
        enable_can_push: 开启CAN推送
    
    位描述:
    
        Byte 0: 控制模式     uint8
                0x00 待机模式
                0x01 CAN 指令控制模式
                0x03 以太网控制模式
                0x04 wifi 控制模式
                0x07 离线轨迹模式
                0x08 TCP 控制模式
        Byte 1: MOVE模式     uint8    
                0x00 MOVE P
                0x01 MOVE J
                0x02 MOVE L
                0x03 MOVE C
                0x04 MOVE MIT ---基于V1.5-2版本后
                0x05 MOVE CPV ---基于V1.8-1版本后
        Byte 2: 运动速度百分比 uint8    0~100
        Byte 3: mit模式      uint8   
                0x00 位置速度模式
                0xAD MIT模式
                0xFF 无效
        Byte 4: 离线轨迹点停留时间 uint8 0~254 ,单位: s;255:轨迹终止
        Byte 5: 安装位置 uint8 注意接线朝后 ---基于V1.5-2版本后
                0x00 无效值
                0x01 水平正装
                0x02 侧装左
                0x03 侧装右
        Byte 6: 开启CAN推送 uint8
                0x00 无效值
                0x01 开启
                0x02 关闭
    '''
    '''
    transmit
    
    Robotic Arm Mode Control Command

    CAN ID:
        0x151
    
    Args:
        ctrl_mode: Control mode.
        move_mode: MOVE mode.
        move_spd_rate_ctrl: Movement speed as a percentage.
        mit_mode: MIT mode.
        residence_time: Hold time at offline trajectory points.
        installation_pos: Installation position.
        enable_can_push: Enable CAN push.

    Bit Descriptions:

        Byte 0 control_mode: uint8, control mode selection.
            0x00: Standby mode.
            0x01: CAN command control mode.
            0x03: Ethernet control mode.
            0x04: Wi-Fi control mode.
            0x07: Offline trajectory mode.
            0x08: TCP control mode.

        Byte 1 move_mode: uint8, movement mode selection.
            0x00: MOVE P (Position).
            0x01: MOVE J (Joint).
            0x02: MOVE L (Linear).
            0x03: MOVE C (Circular).
            0x04: MOVE MIT ---- Based on version V1.5-2 and later
            0x05: MOVE CPV ---- Based on version V1.8-1 and later

        Byte 2 speed_percentage: uint8, movement speed as a percentage.
            Range: 0~100.

        Byte 3 mit_mode: uint8, motion control mode.
            0x00: Position-speed mode.
            0xAD: MIT mode.
            0xFF: Invalid.

        Byte 4 offline_trajectory_hold_time: uint8, duration to hold at offline trajectory points.
            Range: 0~255, unit: seconds.
        
        Byte 5: Installation Position (uint8) - Note: Wiring should face backward ---- Based on version V1.5-2 and later
                0x00: Invalid value
                0x01: Horizontal upright
                0x02: Left-side mount
                0x03: Right-side mount
        
        Byte 6: Enable CAN push (uint8)
                0x00: Invalid value
                0x01: Enable
                0x02: Disable
    '''
    _VALID_CTRL_MODE = [0x00, 0x01, 0x03, 0x04, 0x07, 0x08]
    class Enums:
        @unique
        class CtrlMode(IntEnumBase):
            STANDBY = 0x00
            CAN_CTRL = 0x01
            ETHERNET_CONTROL_MODE = 0x03
            WIFI_CONTROL_MODE = 0x04
            OFFLINE_TRAJECTORY_MODE = 0x07
            TCP_CTRL = 0x08
            UNKNOWN = 0xFF
        @unique
        class MotionMode(IntEnumBase):
            P = 0x00
            J = 0x01
            L = 0x02
            C = 0x03
            MIT = 0x04
            CPV = 0x05
            UNKNOWN = 0xFF
        @unique
        class MitMode(IntEnumBase):
            POS_VEL = 0x00
            MIT = 0xAD
            UNKNOWN = 0xFF
        @unique
        class InstallationPos(IntEnumBase):
            INVALID = 0x00
            HORIZONTAL = 0x01
            LEFT = 0x02
            RIGHT = 0x03
            UNKNOWN = 0xFF
        @unique
        class CanActiveMsgReporting(IntEnumBase):
            INVALID = 0x00
            ENABLE = 0x01
            DISABLE = 0x02
            UNKNOWN = 0xFF
    def __init__(self, 
                 ctrl_mode: Literal[0x00, 0x01, 0x03, 0x04, 0x07, 0x08] = 0x01, 
                 move_mode: Literal[0x00, 0x01, 0x02, 0x03, 0x04, 0x05] = 0x01, 
                 move_spd_rate_ctrl: int = 50,
                 mit_mode: Literal[0x00, 0xAD, 0xFF] = 0x00,
                 residence_time: int = 0,
                 installation_pos: Literal[0x00, 0x01, 0x02, 0x03] = 0x00,
                 enable_can_push: Literal[0x00, 0x01, 0x02] = 0x00):
        super().__init__(ctrl_mode, move_mode, move_spd_rate_ctrl, mit_mode, residence_time, installation_pos)
        if enable_can_push not in [0x00, 0x01, 0x02]:
            raise ValueError("Invalid enable_can_push value.")
        self.enable_can_push = enable_can_push
        