#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....piper.default import ArmMsgCrashProtectionRatingConfig as ArmMsgCrashProtectionRatingConfigBase

class ArmMsgCrashProtectionRatingConfig(ArmMsgCrashProtectionRatingConfigBase):
    '''
    transmit
    
    碰撞防护等级设置指令
    
    CAN ID:
        0x47A

    有效值 : 0~8

    等级 0 代表不检测碰撞； 7个关节可以独立设置

    Args:
        joint_1: 关节1的碰撞等级设定
        joint_2: 关节2的碰撞等级设定
        joint_3: 关节3的碰撞等级设定
        joint_4: 关节4的碰撞等级设定
        joint_5: 关节5的碰撞等级设定
        joint_6: 关节6的碰撞等级设定
        joint_7: 关节7的碰撞等级设定
    
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
    transmit
    
    End Effector Speed/Acceleration Parameter Setting Command

    CAN ID:
        0x47A

    Valid Values: 0~8
        Level 0 indicates no collision detection.
        Collision protection levels can be set independently for the seven joints.

    Args:
        joint_1: Collision protection level for Joint 1.
        joint_2: Collision protection level for Joint 2.
        joint_3: Collision protection level for Joint 3.
        joint_4: Collision protection level for Joint 4.
        joint_5: Collision protection level for Joint 5.
        joint_6: Collision protection level for Joint 6.
        joint_7: Collision protection level for Joint 7.

    Bit Description:

        Byte 0: Collision protection level for Joint 1, uint8.
        Byte 1: Collision protection level for Joint 2, uint8.
        Byte 2: Collision protection level for Joint 3, uint8.
        Byte 3: Collision protection level for Joint 4, uint8.
        Byte 4: Collision protection level for Joint 5, uint8.
        Byte 5: Collision protection level for Joint 6, uint8.
        Byte 6: Collision protection level for Joint 7, uint8.
        Byte 7: Reserved.
    '''
    def __init__(self, 
                 joint_1: int = 0xFF,
                 joint_2: int = 0xFF,
                 joint_3: int = 0xFF,
                 joint_4: int = 0xFF,
                 joint_5: int = 0xFF,
                 joint_6: int = 0xFF,
                 joint_7: int = 0xFF
                 ):
        super().__init__(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
        self.joint_7 = joint_7
        