#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....piper.default import ArmMsgFeedbackCrashProtectionRating as ArmMsgFeedbackCrashProtectionRatingBase
from typing import Optional

class ArmMsgFeedbackCrashProtectionRating(ArmMsgFeedbackCrashProtectionRatingBase):
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
        joint_7: 7号关节碰撞防护等级
    
    设定值 : 0~8

    等级 0 代表不检测碰撞； 7个关节可以独立设置

    位描述:

        Byte 0: 1 号关节碰撞防护等级, uint8
        Byte 1: 2 号关节碰撞防护等级, uint8
        Byte 2: 3 号关节碰撞防护等级, uint8
        Byte 3: 4 号关节碰撞防护等级, uint8
        Byte 4: 5 号关节碰撞防护等级, uint8
        Byte 5: 6 号关节碰撞防护等级, uint8
        Byte 6: 7 号关节碰撞防护等级, uint8
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
        joint_7 (int): Collision protection level for joint 7 (0-8)

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
        Byte 6: Collision protection level for joint 7, uint8
        Byte 7: Reserved
    '''
    def __init__(self,
                 joint_1: Optional[int] = None, 
                 joint_2: Optional[int] = None, 
                 joint_3: Optional[int] = None,
                 joint_4: Optional[int] = None,
                 joint_5: Optional[int] = None,
                 joint_6: Optional[int] = None,
                 joint_7: Optional[int] = None
                 ):
        super().__init__(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
        self.joint_7 = joint_7
    
    def clear(self):
        super().clear()
        self.joint_7 = None
