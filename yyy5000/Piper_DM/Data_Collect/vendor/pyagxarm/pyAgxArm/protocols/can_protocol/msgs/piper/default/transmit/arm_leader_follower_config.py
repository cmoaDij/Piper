#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
from ....core.attritube_base import AttributeBase

class ArmMsgLeaderFollowerModeConfig(AttributeBase):
    '''
    transmit
    
    随动 Leader-Follower 模式设置指令
    
    CAN ID:
        0x470
    
    .. note::
        当前固件上 **Byte 1–3（反馈/控制/联动地址偏移）已废弃**，SDK 发送时固定为 0x00；
        以下字段说明保留作历史协议参考。
    
    Args:
        linkage_config: 联动设置指令
        feedback_offset: 反馈指令偏移值
        ctrl_offset: 控制指令偏移值
        linkage_offset: 联动模式控制目标地址偏移值
    
    位描述:
    
        Byte 0 linkage_config: uint8, 联动设置指令。
                                0x00 无效
                                0xFA 设置为示教输入臂（Leader Arm）
                                0xFC 设置为运动输出臂（Follower Arm）
        Byte 1 feedback_offset: uint8, 反馈指令偏移值。
                                0x00 : 不偏移/恢复默认
                                0x10 ：反馈指令基 ID 由 2Ax偏移为 2Bx
                                0x20 ：反馈指令基 ID 由 2Ax偏移为 2Cx
        Byte 2 ctrl_offset: uint8, 控制指令偏移值。
                                0x00 : 不偏移/恢复默认
                                0x10 ：控制指令基 ID 由 15x偏移为 16x
                                0x20 ：控制指令基 ID 由 15x偏移为 17x
        Byte 3 linkage_offset: uint8, 联动模式控制目标地址偏移值。
                                0x00 : 不偏移/恢复默认
                                0x10 : 控制目标地址基 ID由 15x 偏移为 16x
                                0x20 : 控制目标地址基 ID由 15x 偏移为 17x
    '''
    '''
    transmit

    Follow Leader-Follower Mode Setting Command

    CAN ID:
        0x470

    .. note::
        On current firmware, **bytes 1–3 (offset fields) are deprecated**; the driver
        always transmits **0x00** for each. Field descriptions below are kept for legacy docs.

    Args:
        linkage_config: Linkage setting command.
        feedback_offset: Offset value for feedback instructions.
        ctrl_offset: Offset value for control instructions.
        linkage_offset: Offset value for linkage mode control target address.

    Bit Descriptions:

        Byte 0 linkage_config: uint8, linkage setting command.
            0x00: Invalid.
            0xFA: Set as teaching input arm (Leader Arm).
            0xFC: Set as motion output arm (Follower Arm).

        Byte 1 feedback_offset: uint8, feedback instruction offset value.
            0x00: No offset/restore default.
            0x10: Feedback instruction base ID shifted from 2Ax to 2Bx.
            0x20: Feedback instruction base ID shifted from 2Ax to 2Cx.

        Byte 2 ctrl_offset: uint8, control instruction offset value.
            0x00: No offset/restore default.
            0x10: Control instruction base ID shifted from 15x to 16x.
            0x20: Control instruction base ID shifted from 15x to 17x.

        Byte 3 linkage_offset: uint8, offset value for the linkage mode control target address.
            0x00: No offset/restore default.
            0x10: Control target address base ID shifted from 15x to 16x.
            0x20: Control target address base ID shifted from 15x to 17x.
    '''
    def __init__(self, 
                 linkage_config: Literal[0x00, 0xFA, 0xFC] = 0x00,
                 feedback_offset: Literal[0x00, 0x10, 0x20] = 0x00,
                 ctrl_offset: Literal[0x00, 0x10, 0x20] = 0x00,
                 linkage_offset: Literal[0x00, 0x10, 0x20] = 0x00):
        if linkage_config not in [0x00, 0xFA, 0xFC]:
            raise ValueError(f"'linkage_config' Value {linkage_config} out of range [0x00, 0xFA, 0xFC]")
        if feedback_offset not in [0x00, 0x10, 0x20]:
            raise ValueError(f"'feedback_offset' Value {feedback_offset} out of range [0x00, 0x10, 0x20]")
        if ctrl_offset not in [0x00, 0x10, 0x20]:
            raise ValueError(f"'ctrl_offset' Value {ctrl_offset} out of range [0x00, 0x10, 0x20]")
        if linkage_offset not in [0x00, 0x10, 0x20]:
            raise ValueError(f"'linkage_offset' Value {linkage_offset} out of range [0x00, 0x10, 0x20]")
        self.linkage_config = linkage_config
        self.feedback_offset = feedback_offset
        self.ctrl_offset = ctrl_offset
        self.linkage_offset = linkage_offset
