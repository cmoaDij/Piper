#!/usr/bin/env python3
# -*-coding:utf8-*-
from .....core.attritube_base import AttributeBase

class FeedbackFingerCurrent(AttributeBase):
    '''
    msg_feedback
    
    灵巧手各指电流反馈, 20hz

    CAN ID:
        0x1C3

    Args:
        thumb_tip: 
        thumb_base: 
        index_finger: 
        middle_finger: 
        ring_finger: 
        pinky_finger: 
    
    位描述:
    
        Byte 0:预留
        Byte 1:预留
        Byte 2:
        Byte 3:
        Byte 4:
        Byte 5:
        Byte 6:
        Byte 7:
    '''
    def __init__(self, 
                 thumb_tip: int = -1,
                 thumb_base: int = -1,
                 index_finger: int = -1, 
                 middle_finger: int = -1,
                 ring_finger: int = -1,
                 pinky_finger: int = -1,
                 ):
        self.thumb_tip = thumb_tip
        self.thumb_base = thumb_base
        self.index_finger = index_finger
        self.middle_finger = middle_finger
        self.ring_finger = ring_finger
        self.pinky_finger = pinky_finger
