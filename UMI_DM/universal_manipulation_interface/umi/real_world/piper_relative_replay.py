"""Geometry helpers for replaying a recorded TCP trajectory relatively.

The recorded poses may live in a tag/SLAM frame.  Replay deliberately removes
that global frame by anchoring the trajectory at the robot TCP pose measured at
replay start::

    T_base_tcp_replay(t) = T_base_tcp_start @ inv(T_recorded_tcp_start)
                             @ T_recorded_tcp(t)

This module contains no robot or SDK code so the transform convention can be
tested without hardware.
"""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from umi.common.pose_util import mat_to_pose, pose_to_mat


def _check_pose_array(pose, name="pose"):
    value = np.asarray(pose, dtype=float)
    if value.ndim != 2 or value.shape[1] != 6 or not np.isfinite(value).all():
        raise ValueError(f"{name} must have shape (N, 6) and contain finite values")
    if len(value) == 0:
        raise ValueError(f"{name} must not be empty")
    return value


def _check_rigid(matrix, name):
    value = np.asarray(matrix, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(value[:3, :3]), 1.0, atol=1e-6)):
        raise ValueError(f"{name} must be a rigid transform")
    return value


def pose_array_to_matrices(pose):
    """Convert an ``(N, 6)`` xyz/rotation-vector array to ``(N, 4, 4)``."""
    return pose_to_mat(_check_pose_array(pose))


def sdk_flange_pose_to_matrix(pose):
    """Convert Piper SDK ``[x, y, z, roll, pitch, yaw]`` feedback to a matrix.

    The SDK flange feedback uses XYZ extrinsic Euler angles, while recorded
    UMI poses use rotation vectors.  Keeping this conversion separate avoids
    interpreting SDK roll/pitch/yaw values as a rotation vector.
    """
    value = np.asarray(pose, dtype=float)
    if value.shape != (6,) or not np.isfinite(value).all():
        raise ValueError("SDK flange pose must be six finite xyz/rpy values")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", value[3:]).as_matrix()
    matrix[:3, 3] = value[:3]
    return matrix


def relative_pose_matrices(recorded_pose):
    """Return each recorded pose as a transform relative to the first pose."""
    matrices = pose_array_to_matrices(recorded_pose)
    start_inv = np.linalg.inv(matrices[0])
    relative = np.einsum("ij,njk->nik", start_inv, matrices)
    return relative


def anchor_relative_pose(anchor_tcp, recorded_pose):
    """Apply a recorded relative trajectory to a current base-frame TCP pose."""
    anchor_tcp = _check_rigid(anchor_tcp, "anchor_tcp")
    relative = relative_pose_matrices(recorded_pose)
    return np.einsum("ij,njk->nik", anchor_tcp, relative)


def resample_pose_trajectory(recorded_pose, gripper_width, source_hz, target_hz):
    """Resample pose and gripper data at a command rate using SLERP for rotation."""
    pose = _check_pose_array(recorded_pose, "recorded_pose")
    width = np.asarray(gripper_width, dtype=float).reshape(-1)
    if len(width) != len(pose) or not np.isfinite(width).all():
        raise ValueError("gripper_width must contain one finite value per pose")
    if not np.isfinite([source_hz, target_hz]).all() or source_hz <= 0 or target_hz <= 0:
        raise ValueError("source_hz and target_hz must be positive")
    if len(pose) == 1:
        return pose.copy(), width.copy()

    source_t = np.arange(len(pose), dtype=float) / float(source_hz)
    target_t = np.arange(0.0, source_t[-1] + 1e-10, 1.0 / float(target_hz))
    if len(target_t) == 0 or target_t[-1] < source_t[-1] - 1e-10:
        target_t = np.r_[target_t, source_t[-1]]
    else:
        target_t[-1] = source_t[-1]

    position = np.column_stack([
        np.interp(target_t, source_t, pose[:, axis]) for axis in range(3)
    ])
    rotations = Slerp(source_t, Rotation.from_rotvec(pose[:, 3:]))(target_t)
    output = np.concatenate([position, rotations.as_rotvec()], axis=1)
    output_width = np.interp(target_t, source_t, width)
    return output, output_width


def matrix_to_pose(matrix):
    """Convert one or more rigid transforms to xyz/rotation-vector poses."""
    value = np.asarray(matrix, dtype=float)
    if value.shape == (4, 4):
        _check_rigid(value, "matrix")
    elif value.ndim == 3 and value.shape[1:] == (4, 4):
        for index, item in enumerate(value):
            _check_rigid(item, f"matrix[{index}]")
    else:
        raise ValueError("matrix must have shape (4,4) or (N,4,4)")
    return mat_to_pose(value)
