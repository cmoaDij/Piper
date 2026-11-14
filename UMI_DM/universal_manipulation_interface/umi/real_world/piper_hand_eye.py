"""ChArUco eye-in-hand geometry; T_parent_child maps child into parent.

Camera rotations use OpenCV optical axes: X right, Y down, Z forward.
Piper SDK flange angles are fixed-frame XYZ Euler angles, not rotation vectors.
"""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from umi.common.cv_util import (
    convert_fisheye_intrinsics_resolution, parse_fisheye_intrinsics)


def sdk_flange_to_matrix(pose):
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError('Expected SDK flange [x, y, z, roll, pitch, yaw] in m/rad')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def stable_joint_window(rows, duration_s=1., max_range_deg=.1, max_gap_s=.1):
    """Return a complete stationary window using fresh joint packet times.

    Defaults are configurable capture tolerances: at most 0.1 degree peak to
    peak per joint over one second. Cached SDK reads do not extend the window.
    """
    if duration_s <= 0 or max_range_deg <= 0 or max_gap_s <= 0:
        raise ValueError('Stability duration, range and feedback gap must be positive')
    if not rows:
        return [], 'waiting for feedback'
    stamps = np.asarray([r['joint_monotonic_s'] for r in rows])
    if not np.isfinite(stamps).all() or np.any(np.diff(stamps) < 0):
        return [], 'joint timestamp discontinuity'
    _, indices = np.unique(stamps, return_index=True)
    stamps = stamps[indices]
    cutoff = stamps[-1] - duration_s
    start = int(np.searchsorted(stamps, cutoff, side='right')) - 1
    if start < 0:
        return [], 'waiting for stable window'
    window = [rows[i] for i in indices[start:]]
    if np.any(np.diff(stamps[start:]) > max_gap_s):
        return [], 'feedback gap in stable window'
    joints = np.asarray([r['joint_rad'] for r in window])
    if joints.shape != (len(window), 6) or not np.isfinite(joints).all():
        return [], 'invalid joint feedback'
    if np.max(np.rad2deg(np.ptp(joints, axis=0))) > max_range_deg:
        return [], 'arm moving'
    return window, 'stable'


def is_new_joint_pose(joints, saved_poses, min_change_deg=2.):
    """A new pose differs by at least min_change_deg on one joint from EACH saved pose."""
    joints = np.asarray(joints, dtype=float)
    if joints.shape != (6,) or not np.isfinite(joints).all() or min_change_deg <= 0:
        raise ValueError('Need six finite joint angles and a positive pose separation')
    return all(np.max(np.abs(np.rad2deg(joints - np.asarray(saved)))) >= min_change_deg
               for saved in saved_poses)


def interpolate_flange_feedback(rows, timestamp):
    """Interpolate on SDK receive times; absolute feedback delay is unmeasured."""
    stamps = np.array([r['flange_monotonic_s'] for r in rows])
    # SDK cached packets can be read more than once.
    _, indices = np.unique(stamps, return_index=True)
    rows = [rows[i] for i in indices]
    stamps = stamps[indices]
    if len(stamps) < 2 or not stamps[0] <= timestamp <= stamps[-1]:
        raise ValueError('Camera exposure estimate is outside the recorded feedback window')
    right = min(int(np.searchsorted(stamps, timestamp, side='right')), len(stamps) - 1)
    left = right - 1
    alpha = (timestamp - stamps[left]) / (stamps[right] - stamps[left])
    a, b = [sdk_flange_to_matrix(rows[i]['flange_xyz_rpy']) for i in (left, right)]
    result = np.eye(4)
    result[:3, 3] = a[:3, 3] + alpha * (b[:3, 3] - a[:3, 3])
    result[:3, :3] = Slerp([0., 1.], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))([alpha]).as_matrix()[0]
    return result, {'left_monotonic_s': float(stamps[left]),
                    'right_monotonic_s': float(stamps[right]), 'alpha': float(alpha)}


def select_board_markers(corners, ids, board, reprojection_px=3.):
    """Reject unrelated table tags, including duplicates of board IDs.

    Work in the rectified image, where all board corners share a homography.
    The RANSAC tolerance is a pixel fitting tolerance, not a calibration gate.
    """
    if ids is None:
        return [], None
    lookup = {int(i): p[:, :2] for i, p in zip(board.getIds(), board.getObjPoints())}
    candidates = [(c, int(i)) for c, i in zip(corners, ids.ravel()) if int(i) in lookup]
    if len(candidates) < 2:
        return [], None
    source = np.concatenate([lookup[i] for _, i in candidates])
    target = np.concatenate([c.reshape(4, 2) for c, _ in candidates])
    homography, _ = cv2.findHomography(source, target, cv2.RANSAC, reprojection_px)
    if homography is None:
        return [], None
    fitted = cv2.perspectiveTransform(source.reshape(-1, 1, 2), homography).reshape(-1, 2)
    corner_errors = np.linalg.norm(fitted - target, axis=1).reshape(-1, 4)
    errors = corner_errors.mean(axis=1)
    selected = {}
    for index, (c, marker_id) in enumerate(candidates):
        # Recompute support after OpenCV's final homography refinement; its
        # returned RANSAC mask can refer to an earlier, poorer hypothesis.
        if np.count_nonzero(corner_errors[index] <= reprojection_px) < 3:
            continue
        if marker_id not in selected or errors[index] < selected[marker_id][0]:
            selected[marker_id] = (errors[index], c)
    marker_ids = sorted(selected)
    return [selected[i][1] for i in marker_ids], np.asarray(marker_ids, np.int32).reshape(-1, 1)


class CharucoPoseDetector:
    def __init__(self, intrinsics, resolution, *, squares=(5, 7), square_mm=50.,
                 marker_mm=37.5, dictionary='DICT_4X4_50', legacy_pattern=False):
        if min(squares) < 3 or not 0 < marker_mm < square_mm:
            raise ValueError('Need at least 3x3 squares and 0 < marker size < square size')
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.board = cv2.aruco.CharucoBoard(tuple(squares), square_mm / 1000.,
                                          marker_mm / 1000., self.dictionary)
        if hasattr(self.board, 'setLegacyPattern'):
            self.board.setLegacyPattern(legacy_pattern)
        elif legacy_pattern:
            raise ValueError('Legacy board selection requires the newer OpenCV in the capture environment')
        fisheye = convert_fisheye_intrinsics_resolution(
            parse_fisheye_intrinsics(intrinsics), resolution)
        # Keep the central pixel density rather than fitting the entire very
        # wide fisheye FOV into the rectified canvas and shrinking the board.
        self.camera_matrix = fisheye['K'].copy()
        self.maps = cv2.fisheye.initUndistortRectifyMap(
            fisheye['K'], fisheye['D'], np.eye(3), self.camera_matrix,
            tuple(resolution), cv2.CV_16SC2)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    def detect(self, restored_bgr):
        rectified = cv2.remap(restored_bgr, *self.maps, interpolation=cv2.INTER_LINEAR)
        corners, ids, _ = cv2.aruco.detectMarkers(
            rectified, self.dictionary, parameters=self.parameters)
        detected_ids = [] if ids is None else ids.ravel().tolist()
        corners, ids = select_board_markers(corners, ids, self.board)
        result = {'detected_marker_ids': detected_ids,
                  'board_marker_ids': [] if ids is None else ids.ravel().tolist(),
                  'charuco_corner_count': 0, 'pose_valid': False,
                  'rectified_camera_matrix': self.camera_matrix.tolist()}
        annotated = rectified.copy()
        if ids is None or not len(ids):
            return result, annotated
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        count, image_points, corner_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, rectified, self.board, cameraMatrix=self.camera_matrix,
            distCoeffs=np.zeros(5))
        result['charuco_corner_count'] = count
        if corner_ids is None or count < 4:
            return result, annotated
        cv2.aruco.drawDetectedCornersCharuco(annotated, image_points, corner_ids)
        if self.board.checkCharucoCornersCollinear(corner_ids):
            return result, annotated
        object_points = self.board.getChessboardCorners()[corner_ids.ravel()].astype(np.float64)
        success, rotations, translations, _ = cv2.solvePnPGeneric(
            object_points, image_points, self.camera_matrix, np.zeros(5), flags=cv2.SOLVEPNP_IPPE)
        if not success:
            return result, annotated
        candidates = []
        for rotation, translation in zip(rotations, translations):
            camera_points = object_points @ cv2.Rodrigues(rotation)[0].T + translation.reshape(1, 3)
            if np.any(camera_points[:, 2] <= 0):
                continue
            projected, _ = cv2.projectPoints(object_points, rotation, translation,
                                              self.camera_matrix, np.zeros(5))
            error = np.linalg.norm(projected.reshape(-1, 2) - image_points.reshape(-1, 2), axis=1)
            candidates.append((float(np.mean(error ** 2)), rotation, translation))
        if not candidates:
            return result, annotated
        _, rotation, translation = min(candidates, key=lambda x: x[0])
        rotation, translation = cv2.solvePnPRefineLM(
            object_points, image_points, self.camera_matrix, np.zeros(5), rotation, translation)
        matrix = np.eye(4)
        matrix[:3, :3] = cv2.Rodrigues(rotation)[0]
        matrix[:3, 3] = translation.ravel()
        projected, _ = cv2.projectPoints(object_points, rotation, translation,
                                          self.camera_matrix, np.zeros(5))
        errors = np.linalg.norm(projected.reshape(-1, 2) - image_points.reshape(-1, 2), axis=1)
        result.update({'pose_valid': True, 'T_camera_board': matrix.tolist(),
                       'charuco_ids': corner_ids.ravel().tolist(),
                       'charuco_points_rectified_px': image_points.reshape(-1, 2).tolist(),
                       'reprojection_rms_px': float(np.sqrt(np.mean(errors ** 2))),
                       'reprojection_max_px': float(np.max(errors))})
        cv2.drawFrameAxes(annotated, self.camera_matrix, np.zeros(5), rotation, translation, .075)
        return result, annotated


def solve_hand_eye(base_flange, camera_board):
    """Return T_flange_camera and fixed-board consistency using OpenCV Park."""
    base_flange, camera_board = np.asarray(base_flange), np.asarray(camera_board)
    if (base_flange.shape != camera_board.shape or base_flange.shape[1:] != (4, 4)
            or len(base_flange) < 3 or not np.isfinite(base_flange).all()
            or not np.isfinite(camera_board).all()):
        raise ValueError('Need at least three finite paired 4x4 transforms')
    motions = Rotation.from_matrix(
        np.array([base_flange[0, :3, :3].T @ t[:3, :3] for t in base_flange[1:]])).as_rotvec()
    if np.linalg.matrix_rank(motions, tol=1e-8) < 2:
        raise ValueError('Unobservable hand-eye geometry: rotate about at least two nonparallel axes')
    rotation, translation = cv2.calibrateHandEye(
        [t[:3, :3] for t in base_flange], [t[:3, 3] for t in base_flange],
        [t[:3, :3] for t in camera_board], [t[:3, 3] for t in camera_board],
        method=cv2.CALIB_HAND_EYE_PARK)
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation.ravel()
    if not np.isfinite(matrix).all() or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6):
        raise ValueError('Hand-eye solver returned an invalid rotation; acquire more diverse poses')
    boards = base_flange @ matrix @ camera_board
    center = np.eye(4)
    center[:3, 3] = np.mean(boards[:, :3, 3], axis=0)
    center[:3, :3] = Rotation.from_matrix(boards[:, :3, :3]).mean().as_matrix()
    translation_errors = np.linalg.norm(boards[:, :3, 3] - center[:3, 3], axis=1) * 1000
    rotation_errors = np.rad2deg(Rotation.from_matrix(
        np.array([center[:3, :3].T @ t[:3, :3] for t in boards])).magnitude())
    diagnostics = {'T_base_board_mean': center.tolist(),
                   'board_position_residual_mm': translation_errors.tolist(),
                   'board_rotation_residual_deg': rotation_errors.tolist(),
                   'rotation_motion_singular_values_rad': np.linalg.svd(motions, compute_uv=False).tolist(),
                   'flange_position_span_mm': (np.ptp(base_flange[:, :3, 3], axis=0) * 1000).tolist(),
                   'max_relative_rotation_deg': float(np.rad2deg(np.max(np.linalg.norm(motions, axis=1))))}
    return matrix, diagnostics
