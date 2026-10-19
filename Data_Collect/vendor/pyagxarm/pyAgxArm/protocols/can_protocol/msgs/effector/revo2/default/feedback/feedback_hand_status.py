#!/usr/bin/env python3
# -*-coding:utf8-*-
from .....core.attritube_base import AttributeBase

class FeedbackHandStatus(AttributeBase):
    '''
    msg_feedback
    
    灵巧手各指状态反馈, 20hz

    CAN ID:
        0x1C0

    Args:
        left_or_right: 左右手标志, 01左手; 02右手
        thumb_tip: 拇指尖, 0 马达空闲; 1 马达运行; 2 马达堵转
        thumb_base: 拇指根, 0 马达空闲; 1 马达运行; 2 马达堵转
        index_finger: 食指, 0 马达空闲; 1 马达运行; 2 马达堵转
        middle_finger: 中指, 0 马达空闲; 1 马达运行; 2 马达堵转
        ring_finger: 无名指, 0 马达空闲; 1 马达运行; 2 马达堵转
        pinky_finger: 小指, 0 马达空闲; 1 马达运行; 2 马达堵转
    
    位描述:

        Byte 0: 左右手标志, uint8_t
                01左手;
                02右手
        Byte 1: 预留
        Byte 2: 拇指尖, uint8_t
                0 马达空闲
                1 马达运行
                2 马达堵转
        Byte 3: 拇指根, uint8_t
                0 马达空闲
                1 马达运行
                2 马达堵转
        Byte 4: 食指, uint8_t
                0 马达空闲
                1 马达运行
                2 马达堵转
        Byte 5: 中指, uint8_t
                0 马达空闲
                1 马达运行
                2 马达堵转
        Byte 6: 无名指, uint8_t
                0 马达空闲
                1 马达运行
                2 马达堵转
        Byte 7: 小指, uint8_t
                0 马达空闲
                1 马达运行
                2 马达堵转
    '''
    def __init__(self, 
                left_or_right: int = 0,
                thumb_tip: int = -1,
                thumb_base: int = -1,
                index_finger: int = -1,
                middle_finger: int = -1,
                ring_finger: int = -1,
                pinky_finger: int = -1
                ):
        self.left_or_right = left_or_right
        self.thumb_tip = thumb_tip
        self.thumb_base = thumb_base
        self.index_finger = index_finger
        self.middle_finger = middle_finger
        self.ring_finger = ring_finger
        self.pinky_finger = pinky_finger
