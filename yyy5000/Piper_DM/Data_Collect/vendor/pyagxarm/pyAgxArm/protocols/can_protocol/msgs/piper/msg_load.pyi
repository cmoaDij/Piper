# pyAgxArm/protocols/can_protocol/msgs/piper/piper_msg_load.pyi
from .default import *
from typing import Optional

class FeedbackNS:
    ArmMsgFeedbackJointStates: ArmMsgFeedbackJointStates
    ArmMsgFeedbackEndPose: ArmMsgFeedbackEndPose
    # ...只写你希望 IDE 补全的

class TransmitNS:
    ArmMsgJointCtrl: ArmMsgJointCtrl
    ArmMsgEndPoseCtrl: ArmMsgEndPoseCtrl
    # ...

class MsgBundle:
    feedback: FeedbackNS
    transmit: TransmitNS

def load_msgs(
    arm: str,
    version: Optional[str] = None,
) -> MsgBundle: ...
