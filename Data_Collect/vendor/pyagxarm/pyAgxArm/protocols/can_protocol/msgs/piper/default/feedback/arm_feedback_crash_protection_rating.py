#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase
from typing import Optional

class ArmMsgFeedbackCrashProtectionRating(AttributeBase):
    '''
    feedback
    
    碰撞防护等级反馈指令
    
    0x477 Byte 0 = 0x02 feedback
    
    CAN ID: 
        0x47B
    
    Args:
        joint_1: 1号关节碰撞防护等级
        joint_2: 2号关节碰撞防护等级
        joint_3: 3号关节碰撞防护等级
        joint_4: 4号关节碰撞防护等级
        joint_5: 5号关节碰撞防护等级
        joint_6: 6号关节碰撞防护等级
    
    设定值 : 0~8

    等级 0 代表不检测碰撞； 6个关节可以独立设置

    位描述:

        Byte 0: 1 号关节碰撞防护等级, uint8
        Byte 1: 2 号关节碰撞防护等级, uint8
        Byte 2: 3 号关节碰撞防护等级, uint8
        Byte 3: 4 号关节碰撞防护等级, uint8
        Byte 4: 5 号关节碰撞防护等级, uint8
        Byte 5: 6 号关节碰撞防护等级, uint8
        Byte 6: 保留
        Byte 7: 保留
    '''
    
    '''
    feedback
    
    Get the collision protection level feedback for each joint.

    0x477 Byte 0 = 0x02 feedback
    
    CAN ID: 
        0x47B

    Args:
        joint_1 (int): Collision protection level for joint 1 (0-8)
        joint_2 (int): Collision protection level for joint 2 (0-8)
        joint_3 (int): Collision protection level for joint 3 (0-8)
        joint_4 (int): Collision protection level for joint 4 (0-8)
        joint_5 (int): Collision protection level for joint 5 (0-8)
        joint_6 (int): Collision protection level for joint 6 (0-8)

    Level Description:
        0: No collision detection
        1-8: Collision detection thresholds increase (higher values represent more sensitive thresholds)

    Byte Description:

        Byte 0: Collision protection level for joint 1, uint8
        Byte 1: Collision protection level for joint 2, uint8
        Byte 2: Collision protection level for joint 3, uint8
        Byte 3: Collision protection level for joint 4, uint8
        Byte 4: Collision protection level for joint 5, uint8
        Byte 5: Collision protection level for joint 6, uint8
        Byte 6: Reserved
        Byte 7: Reserved
    '''
    def __init__(self, 
                 joint_1: Optional[int] = None,
                 joint_2: Optional[int] = None,
                 joint_3: Optional[int] = None,
                 joint_4: Optional[int] = None,
                 joint_5: Optional[int] = None,
                 joint_6: Optional[int] = None
                 ):
        self.joint_1 = joint_1
        self.joint_2 = joint_2
        self.joint_3 = joint_3
        self.joint_4 = joint_4
        self.joint_5 = joint_5
        self.joint_6 = joint_6
    
    def clear(self):
        self.joint_1 = None
        self.joint_2 = None
        self.joint_3 = None
        self.joint_4 = None
        self.joint_5 = None
        self.joint_6 = None
