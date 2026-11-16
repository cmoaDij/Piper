"""Numerical tests for marker-free relative replay geometry."""

import numpy as np
from scipy.spatial.transform import Rotation

from umi.real_world.piper_relative_replay import (
    anchor_relative_pose,
    matrix_to_pose,
    pose_array_to_matrices,
    relative_pose_matrices,
    resample_pose_trajectory,
    sdk_flange_pose_to_matrix,
)


def test_relative_replay_identity_and_anchor():
    recorded = np.array([
        [0.2, -0.1, 0.4, 0.0, 0.0, 0.0],
        [0.25, -0.1, 0.4, 0.0, 0.0, np.deg2rad(20.0)],
    ])
    anchor = np.eye(4)
    anchor[:3, :3] = Rotation.from_euler("xyz", [10.0, -5.0, 30.0], degrees=True).as_matrix()
    anchor[:3, 3] = [0.4, 0.1, 0.35]
    target = anchor_relative_pose(anchor, recorded)
    assert np.allclose(target[0], anchor)
    assert np.allclose(target[1], anchor @ np.linalg.inv(pose_array_to_matrices(recorded)[0]) @ pose_array_to_matrices(recorded)[1])


def test_resample_uses_slerp_and_keeps_endpoint():
    recorded = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0, 0.0, 0.0, np.pi],
    ])
    pose, width = resample_pose_trajectory(recorded, [0.0, 0.06], 10.0, 20.0)
    assert len(pose) == 3
    assert np.allclose(pose[0], recorded[0])
    assert np.allclose(pose[-1], recorded[-1])
    assert np.allclose(width, [0.0, 0.03, 0.06])
    matrices = relative_pose_matrices(pose)
    assert np.allclose(matrix_to_pose(matrices[0]), np.zeros(6))


def test_rejects_invalid_pose():
    try:
        pose_array_to_matrices(np.zeros((0, 6)))
    except ValueError:
        pass
    else:
        raise AssertionError("empty trajectory should be rejected")


def test_sdk_flange_feedback_uses_xyz_extrinsic_euler_angles():
    pose = np.array([0.1, -0.2, 0.3, 0.4, -0.5, 0.6])
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler("xyz", pose[3:]).as_matrix()
    expected[:3, 3] = pose[:3]
    assert np.allclose(sdk_flange_pose_to_matrix(pose), expected)
    wrong = np.eye(4)
    wrong[:3, :3] = Rotation.from_rotvec(pose[3:]).as_matrix()
    wrong[:3, 3] = pose[:3]
    assert not np.allclose(sdk_flange_pose_to_matrix(pose), wrong)
