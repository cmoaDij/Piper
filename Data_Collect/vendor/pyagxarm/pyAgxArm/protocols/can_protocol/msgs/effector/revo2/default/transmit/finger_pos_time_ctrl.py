#!/usr/bin/env python3
# -*-coding:utf8-*-
from .....core.attritube_base import AttributeBase


class FingerPosTimeCtrl(AttributeBase):
    """
    transmit

    手指位置时间控制指令

    Finger Position or Time Control Command

    CAN ID:
        0x1B5

    Notes
    -----
    Payload layout (8 bytes):
    - Byte 0: 0x00 (reserved)
    - Byte 1: mode (uint8)
    - Byte 2: thumb_tip (uint8)
    - Byte 3: thumb_base (uint8)
    - Byte 4: index_finger (uint8)
    - Byte 5: middle_finger (uint8)
    - Byte 6: ring_finger (uint8)
    - Byte 7: pinky_finger (uint8)

    Args:
        mode: Control mode, 0x12: position control, 0x22: time control
        thumb_tip: position range 0 to 100, time range 0 to 255, default 0
        thumb_base: position range 0 to 100, time range 0 to 255, default 0
        index_finger: position range 0 to 100, time range 0 to 255, default 0
        middle_finger: position range 0 to 100, time range 0 to 255, default 0
        ring_finger: position range 0 to 100, time range 0 to 255, default 0
        pinky_finger: position range 0 to 100, time range 0 to 255, default 0
    """

    def __init__(
        self,
        mode: int = 0x12,
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0
    ):
        self.mode = mode
        if self.mode not in [0x12, 0x22]:
            raise ValueError("Invalid mode, shoule be 0x12 or 0x22.")

        if self.mode == 0x12:
            upper = 100
        elif self.mode == 0x22:
            upper = 255

        self.thumb_tip = max(0, min(thumb_tip, upper))
        self.thumb_base = max(0, min(thumb_base, upper))
        self.index_finger = max(0, min(index_finger, upper))
        self.middle_finger = max(0, min(middle_finger, upper))
        self.ring_finger = max(0, min(ring_finger, upper))
        self.pinky_finger = max(0, min(pinky_finger, upper))


__all__ = ["FingerPosTimeCtrl"]
