"""URDF forward kinematics using the same model and TCP offset as teleoperation."""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def transform(xyz, rpy):
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    mat[:3, 3] = xyz
    return mat


class ForwardKinematics:
    def __init__(self, urdf, base="base_link", tip="link6", tcp_offset=(0,) * 6):
        tree = ET.parse(urdf).getroot()
        by_child = {j.find("child").get("link"): j for j in tree.findall("joint")}
        self.chain = []
        seen = set()
        while tip != base:
            if tip in seen or tip not in by_child:
                raise ValueError("URDF 中找不到完整的 base 到 tip 链")
            seen.add(tip)
            joint = by_child[tip]
            origin = joint.find("origin")
            xyz = np.fromstring(
                origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" "
            )
            rpy = np.fromstring(
                origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" "
            )
            axis = joint.find("axis")
            vec = np.fromstring(axis.get("xyz", "1 0 0") if axis is not None else "1 0 0", sep=" ")
            self.chain.append((joint.get("name"), joint.get("type"), transform(xyz, rpy), vec))
            tip = joint.find("parent").get("link")
        self.chain.reverse()
        names = [j[0] for j in self.chain if j[1] != "fixed"]
        if names != [f"joint{i}" for i in range(1, 7)]:
            raise ValueError(f"需要 joint1..joint6 六轴链，实际为 {names}")
        self.offset = transform(tcp_offset[:3], tcp_offset[3:])

    def matrix(self, joints):
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError("FK 需要有限的六轴关节角")
        mat = np.eye(4)
        q = iter(joints)
        for _, kind, origin, axis in self.chain:
            mat = mat @ origin
            if kind in ("revolute", "continuous"):
                rot = np.eye(4)
                rot[:3, :3] = Rotation.from_rotvec(
                    axis / np.linalg.norm(axis) * next(q)
                ).as_matrix()
                mat = mat @ rot
            elif kind != "fixed":
                raise ValueError(f"不支持关节类型 {kind}")
        return mat @ self.offset

    def __call__(self, joints):
        mat = self.matrix(joints)
        return np.r_[mat[:3, 3], Rotation.from_matrix(mat[:3, :3]).as_euler("xyz")].astype(
            np.float32
        )


def pose_error(model_pose, sdk_pose):
    model_pose, sdk_pose = np.asarray(model_pose, dtype=float), np.asarray(sdk_pose, dtype=float)
    if model_pose.shape != (6,) or sdk_pose.shape != (6,) or not np.isfinite([model_pose, sdk_pose]).all():
        raise ValueError("TCP 比较需要两个有限的 xyz/rpy 位姿")
    position = float(np.linalg.norm(model_pose[:3] - sdk_pose[:3]))
    rotation = float((Rotation.from_euler("xyz", model_pose[3:]).inv() * Rotation.from_euler("xyz", sdk_pose[3:])).magnitude())
    return position, rotation


def check_kinematics(feedback, rules):
    skew = abs(float(feedback["sdk_joint_ts"]) - float(feedback["sdk_tcp_ts"]))
    if skew > rules["feedback_skew_s"]:
        raise ValueError(f"关节/TCP 反馈不同步：{skew * 1000:.1f} ms")
    position, rotation = pose_error(feedback["model_tcp_pose"], feedback["tcp_pose"])
    if position > rules["fk_position_tolerance_m"] or rotation > rules["fk_rotation_tolerance_rad"]:
        raise ValueError(f"模型 FK 与 SDK TCP 不一致：{position * 1000:.1f} mm / {np.rad2deg(rotation):.2f}°，拒绝运动")
    return position, rotation
