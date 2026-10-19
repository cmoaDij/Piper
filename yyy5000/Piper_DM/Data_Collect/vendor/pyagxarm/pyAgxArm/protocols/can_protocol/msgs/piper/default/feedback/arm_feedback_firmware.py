#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase

class ArmMsgFeedbackFirmware(AttributeBase):
    '''CAN ID:
        0x4AF'''
    def __init__(self):
        self.data_seg = bytearray()
        self.data_concat = bytearray()
        self.main_ctrl_firmware = bytearray()
        self.motor_firmware = bytearray()

    def clear(self):
        self.data_seg = bytearray()
        self.data_concat = bytearray()
