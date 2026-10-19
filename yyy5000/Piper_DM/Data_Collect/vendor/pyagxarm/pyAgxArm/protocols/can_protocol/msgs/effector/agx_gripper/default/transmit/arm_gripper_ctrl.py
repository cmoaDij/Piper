#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import Union
from typing_extensions import Literal

from .....core.attritube_base import AttributeBase


class ArmMsgGripperCtrl(AttributeBase):
    """
    transmit

    夹爪控制指令

    Gripper Control Command

    CAN ID:
        0x159

    Notes
    -----
    Payload layout (8 bytes):
    - Byte 0-3: width_µm/angle_mdeg (int32), unit: µm/mdeg
      - width_m = width_µm * 1e-6
      - angle_deg = angle_mdeg * 1e-3
    - Byte 4-5: force_mN (int16), unit: mN
      - force_N = force_mN * 1e-3
    - Byte 6: status_code (uint8)
      - 0x00: disable/width
      - 0x01: enable/width
      - 0x02: disable/clear/width
      - 0x03: enable/clear/width
      - 0x04: disable/angle
      - 0x05: enable/angle
      - 0x06: disable/clear/angle
      - 0x07: enable/clear/angle
    - Byte 7: set_zero 
      - 0xAE: set current position as zero
      - 0x00: no-op

    Args:
        value: raw integer width/angle in µm/mdeg (NOT meters/degrees).
        force: raw integer force in mN (NOT Newton).
        status_code: see above.
        set_zero: 0xAE triggers zeroing.
    """

    def __init__(
        self,
        value: Union[int, float] = 0,
        force: Union[int, float] = 0,
        status_code: Literal[0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07] = 0,
        set_zero: Literal[0x00, 0xAE] = 0,
    ):
        if status_code not in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
            raise ValueError(
                f"'status_code' Value {status_code} out of range "
                "[0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]"
            )
        if set_zero not in [0x00, 0xAE]:
            raise ValueError(
                f"'set_zero' Value {set_zero} out of range [0x00,0xAE]"
            )
        self.value = value
        self.force = force
        self.status_code = status_code
        self.set_zero = set_zero


__all__ = ["ArmMsgGripperCtrl"]
