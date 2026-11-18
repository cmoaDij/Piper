"""Quest LOCAL-space protocol and fixed, gravity-aligned robot mapping.

The native sender supplies grip and aim poses directly in OpenXR LOCAL space.
No headset pose enters this calculation. Heading is captured once, separately
from the hand/TCP origins captured by A and RG.
"""

import json
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation


PROTOCOL = "piper.quest.local.v1"
PACKAGE = "com.piper.questinput"
LOG_TAG = "PiperQuestWorld"
POSE_TIMEOUT_S = 0.5


def pose_matrix(values):
    """Decode [x,y,z,qx,qy,qz,qw], metres and OpenXR quaternion order."""
    pose = np.asarray(values, dtype=float)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError("Quest pose must contain seven finite values")
    if not np.isclose(np.linalg.norm(pose[3:]), 1.0, atol=1e-3):
        raise ValueError("Quest quaternion is not normalized")
    result = np.eye(4)
    result[:3, 3] = pose[:3]
    result[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    return result


def decode_packet(value):
    packet = json.loads(value) if isinstance(value, str) else dict(value)
    if packet.get("protocol") != PROTOCOL or packet.get("space") != "LOCAL":
        raise ValueError("需要 Quest LOCAL 世界坐标协议，不能使用旧的头显相对坐标")
    for key in ("session", "generation", "seq", "device_monotonic_ns"):
        if type(packet.get(key)) is not int or packet[key] < 0:
            raise ValueError(f"Invalid Quest {key}")
    if type(packet.get("valid")) is not bool:
        raise ValueError("Missing Quest tracking validity")
    buttons = packet.get("buttons")
    if not isinstance(buttons, dict):
        raise ValueError("Missing Quest buttons")
    for key in ("A", "B", "RJ", "RG"):
        if type(buttons.get(key)) is not bool:
            raise ValueError(f"Invalid Quest button {key}")
    trigger = buttons.get("rightTrig")
    if type(trigger) not in (int, float) or not math.isfinite(trigger) or not 0 <= trigger <= 1:
        raise ValueError("Invalid Quest trigger")
    if packet["valid"]:
        pose_matrix(packet["grip"])
        pose_matrix(packet["aim"])
    return packet


def heading_to_base(aim_pose):
    """Map the controller's horizontal pointing direction (-Z aim) to base +X.

    OpenXR +Y is gravity up. Construct orthonormal forward/left/up rows, so
    changing grip roll/pitch is not mistaken for a direction calibration.
    """
    forward = -aim_pose[:3, 2].copy()
    up = np.array([0.0, 1.0, 0.0])
    forward[1] = 0.0
    length = np.linalg.norm(forward)
    # A nearly vertical pointer cannot determine a horizontal heading. This
    # is a geometric singularity guard, not a tracking quality score.
    if length < 1e-3:
        raise ValueError("手柄指向接近竖直；请水平指向机械臂基座 +X 方向后按 A")
    forward /= length
    left = np.cross(up, forward)
    return np.stack((forward, left, up))


class WorldInputState:
    def __init__(self, timeout_s=POSE_TIMEOUT_S):
        self.timeout_s = timeout_s
        self.packet = None
        self.received_ns = None
        self.token = None
        self.sequence = -1
        self.rotation = None

    def update(self, value, now_ns=None):
        packet = decode_packet(value)
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        stamp = packet.get("host_monotonic_ns")
        if type(stamp) is not int or not 0 <= now_ns - stamp <= self.timeout_s * 1e9:
            raise ValueError("Quest 世界坐标输入已过期")
        token = (packet["session"], packet["generation"])
        changed = self.token is not None and token != self.token
        if token == self.token and packet["seq"] <= self.sequence:
            return False
        if changed:
            self.rotation = None
        self.packet, self.token, self.sequence = packet, token, packet["seq"]
        self.received_ns = stamp
        return changed

    def fresh(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        return (self.received_ns is not None
                and 0 <= now_ns - self.received_ns <= self.timeout_s * 1e9)

    def valid(self, now_ns=None):
        return self.fresh(now_ns) and self.packet["valid"]

    def unavailable_reason(self, now_ns=None):
        """Explain an input rejection without accepting an untracked pose."""
        if self.packet is None:
            return "尚未收到 Quest 世界坐标数据"
        if not self.fresh(now_ns):
            return f"Quest 数据超过 {self.timeout_s * 1000:g} ms 未更新"
        if self.packet["valid"]:
            return None
        tracking = self.packet.get("tracking", {})
        if not isinstance(tracking, dict):
            tracking = {}
        if tracking.get("focused") is False:
            return "Piper Quest World 未获得输入焦点，请先完成游戏边界等系统提示，关闭菜单并回到蓝色场景"
        if tracking.get("grip_active") is False or tracking.get("aim_active") is False:
            return "右手柄未激活，请先在头显内完成游戏边界等系统提示并返回蓝色场景，再按食指扳机唤醒"
        if tracking.get("reference_ready") is False:
            return "Quest 正在重置追踪空间，请等待重新定位完成"
        if tracking.get("grip") is False or tracking.get("aim") is False:
            return "右手柄缺少完整空间追踪，请保持手柄在头显前方且无遮挡"
        return "Quest 世界位姿无效，请恢复头显及右手柄追踪"

    def align_once(self):
        if not self.valid():
            raise ValueError("Quest 世界坐标追踪未就绪")
        if self.rotation is None:
            self.rotation = heading_to_base(pose_matrix(self.packet["aim"]))
        return self.rotation

    def mapped_pose(self):
        if not self.valid() or self.rotation is None:
            return None
        pose = pose_matrix(self.packet["grip"])
        pose[:3, :] = self.rotation @ pose[:3, :]
        return pose

    def metadata(self):
        return {
            "coordinate_mode": "world", "input_space": "OpenXR_LOCAL",
            "session": self.token[0] if self.token else None,
            "generation": self.token[1] if self.token else None,
            "world_to_base_rotation": self.rotation.tolist() if self.rotation is not None else None,
            "heading_definition": "first_A_horizontal_aim_to_base_positive_X",
        }
