#!/usr/bin/env python3
# -*-coding:utf8-*-
from .....core.attritube_base import AttributeBase


class FingerCurrentCtrl(AttributeBase):
    """
    transmit

    手指电流控制指令

    Finger Current Control Command

    CAN ID:
        0x1B3

    Notes
    -----
    Payload layout (8 bytes):
    - Byte 0: 0x00 (reserved)
    - Byte 1: 0x00 (reserved)
    - Byte 2: thumb_tip (uint8)
    - Byte 3: thumb_base (uint8)
    - Byte 4: index_finger (uint8)
    - Byte 5: middle_finger (uint8)
    - Byte 6: ring_finger (uint8)
    - Byte 7: pinky_finger (uint8)

    Args:
        thumb_tip: -100~100, default 0
        thumb_base: -100~100, default 0
        index_finger: -100~100, default 0
        middle_finger: -100~100, default 0
        ring_finger: -100~100, default 0
        pinky_finger: -100~100, default 0
    """

    def __init__(
        self,
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0
    ):
        self.thumb_tip = max(-100, min(thumb_tip, 100))
        self.thumb_base = max(-100, min(thumb_base, 100))
        self.index_finger = max(-100, min(index_finger, 100))
        self.middle_finger = max(-100, min(middle_finger, 100))
        self.ring_finger = max(-100, min(ring_finger, 100))
        self.pinky_finger = max(-100, min(pinky_finger, 100))


__all__ = ["FingerCurrentCtrl"]
