"""Synthetic convention, observability, timing and duplicate-marker regressions."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from umi.real_world.piper_hand_eye import (
    interpolate_flange_feedback, is_new_joint_pose, sdk_flange_to_matrix,
    select_board_markers, solve_hand_eye, stable_joint_window)


def test_auto_capture_waits_for_stationary_joints():
    rows = [{'joint_monotonic_s': float(t), 'joint_rad': [0.] * 6}
            for t in np.linspace(10., 11.2, 121)]
    window, reason = stable_joint_window(rows)
    assert window and reason == 'stable'
    assert window[-1]['joint_monotonic_s'] - window[0]['joint_monotonic_s'] >= 1.
    assert not stable_joint_window(rows[:50])[0]
    # Continuing slow motion and a brief twitch must both reset settling.
    moving = [dict(r, joint_rad=[np.deg2rad(.5 * i / 120)] + [0.] * 5)
              for i, r in enumerate(rows)]
    assert not stable_joint_window(moving)[0]
    rows[70]['joint_rad'][4] = np.deg2rad(.2)
    assert not stable_joint_window(rows)[0]
    stationary = [{'joint_monotonic_s': float(t), 'joint_rad': [0.] * 6}
                  for t in np.linspace(11.21, 12.41, 121)]
    assert stable_joint_window(rows + stationary)[0]


def test_cached_feedback_and_packet_gaps_do_not_trigger_capture():
    cached = [{'joint_monotonic_s': 10., 'joint_rad': [0.] * 6}] * 150
    assert not stable_joint_window(cached)[0]
    gapped = [{'joint_monotonic_s': float(t), 'joint_rad': [0.] * 6}
              for t in np.r_[np.linspace(10., 10.5, 51), np.linspace(10.7, 11.2, 51)]]
    assert stable_joint_window(gapped)[1] == 'feedback gap in stable window'


def test_pose_deduplication_checks_all_saved_poses():
    first, second = np.zeros(6), np.deg2rad([0., 0., 12., 0., 0., 0.])
    assert is_new_joint_pose(first, [])
    assert not is_new_joint_pose(first, [first])
    assert not is_new_joint_pose(first + np.deg2rad([.02] * 6), [first])
    assert is_new_joint_pose(second, [first])
    # Returning to the first pose is skipped even after a different pose was saved.
    assert not is_new_joint_pose(first, [first, second])
    assert is_new_joint_pose(np.deg2rad([3., 0., 0., 0., 0., 0.]), [first, second])


def test_hand_eye_direction_and_tcp_composition():
    rng = np.random.default_rng(42)
    flange_camera = sdk_flange_to_matrix([.02, -.04, .09, .2, -.35, .8])
    fixed_board = sdk_flange_to_matrix([.45, .12, .04, -.5, .2, .1])
    base_flange = [sdk_flange_to_matrix(np.r_[rng.uniform(-.4, .4, 3),
                                             rng.uniform(-1.2, 1.2, 3)]) for _ in range(20)]
    camera_board = [np.linalg.inv(t @ flange_camera) @ fixed_board for t in base_flange]
    recovered, residuals = solve_hand_eye(base_flange, camera_board)
    np.testing.assert_allclose(recovered, flange_camera, atol=1e-10)
    assert max(residuals['board_position_residual_mm']) < 1e-7
    assert max(residuals['board_rotation_residual_deg']) < 1e-7
    camera_tcp = sdk_flange_to_matrix([0, .096, .138, 0, 0, 0])
    np.testing.assert_allclose(recovered @ camera_tcp, flange_camera @ camera_tcp, atol=1e-10)


def test_reject_unobservable_motion():
    flange_camera = sdk_flange_to_matrix([.02, -.04, .09, .2, -.35, .8])
    base_flange = [sdk_flange_to_matrix([0., 0., .3, 0., 0., angle]) for angle in (0., .2, .4, .6)]
    camera_board = [np.linalg.inv(t @ flange_camera) for t in base_flange]
    try:
        solve_hand_eye(base_flange, camera_board)
    except ValueError as exc:
        assert 'nonparallel' in str(exc)
    else:
        raise AssertionError('Single-axis motion cannot determine all hand-eye parameters')


def test_sdk_euler_and_interpolated_feedback():
    a = [.1, .2, .3, .3, -.2, .5]
    b = [.2, .4, .6, .5, -.1, .8]
    matrix = sdk_flange_to_matrix(a)
    np.testing.assert_allclose(matrix[:3, :3], Rotation.from_euler('xyz', a[3:]).as_matrix())
    assert not np.allclose(matrix[:3, :3], Rotation.from_rotvec(a[3:]).as_matrix())
    rows = [{'flange_monotonic_s': t, 'flange_xyz_rpy': pose} for t, pose in [(1., a), (1., a), (2., b)]]
    middle, bracket = interpolate_flange_feedback(rows, 1.5)
    np.testing.assert_allclose(middle[:3, 3], [.15, .3, .45])
    assert bracket['alpha'] == .5
    try:
        interpolate_flange_feedback(rows, .9)
    except ValueError:
        pass
    else:
        raise AssertionError('Out-of-window camera timestamp must not use the newest feedback')


def test_duplicate_table_tags_are_rejected():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((5, 7), .05, .0375, dictionary)
    object_points = board.getObjPoints()
    homography = np.array([[1500., 50., 300.], [20., 1400., 150.], [.1, .2, 1.]])
    corners = [cv2.perspectiveTransform(p[:, :2].reshape(1, 4, 2), homography) for p in object_points]
    # A separate table tag has the same ID as a marker on the board.
    corners.append(np.array([[[30., 30.], [60., 30.], [60., 60.], [30., 60.]]], np.float32))
    ids = np.r_[board.getIds(), [3]].reshape(-1, 1)
    selected, selected_ids = select_board_markers(corners, ids, board)
    assert len(selected) == len(board.getIds())
    assert len(set(selected_ids.ravel())) == len(selected_ids)
    index = selected_ids.ravel().tolist().index(3)
    np.testing.assert_allclose(selected[index], corners[3])


if __name__ == '__main__':
    test_auto_capture_waits_for_stationary_joints()
    test_cached_feedback_and_packet_gaps_do_not_trigger_capture()
    test_pose_deduplication_checks_all_saved_poses()
    test_hand_eye_direction_and_tcp_composition()
    test_reject_unobservable_motion()
    test_sdk_euler_and_interpolated_feedback()
    test_duplicate_table_tags_are_rejected()
    print('Piper hand-eye geometry checks passed')
