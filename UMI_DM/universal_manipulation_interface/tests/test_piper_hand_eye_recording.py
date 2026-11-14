"""Exercise automatic recording/resume with fake read-only devices; no hardware."""
import importlib.util
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np
import yaml


def test_automatic_capture_and_resume(tmp_path):
    spec = importlib.util.spec_from_file_location(
        'piper_hand_eye_cli', Path(__file__).resolve().parents[1] / 'scripts/calibrate_piper_hand_eye.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    setup = tmp_path / 'setup.yaml'
    setup.write_text(yaml.safe_dump({'camera_image_geometry': 'native_4_3',
                                    'camera_output_size': [160, 120], 'nominal_camera_fps': 60,
                                    'camera': 'fake',
                                    'latency_compensation': {'camera_observation_latency_s': .146}}))
    intrinsics = tmp_path / 'intrinsics.json'
    intrinsics.write_text('{}')
    args = SimpleNamespace(command='record', setup=setup, intrinsics=intrinsics, can='fake',
                           camera=None, camera_size=None, camera_fps=None, output=tmp_path / 'poses',
                           squares_x=5, squares_y=7, square_mm=50., marker_mm=37.5,
                           dictionary='DICT_4X4_50', legacy_pattern=False,
                           stable_s=1., stable_deg=.1, min_pose_change_deg=2., once=True)
    image = np.zeros((120, 160, 3), np.uint8)
    joints = np.zeros(6)

    class Arm:
        disconnected = 0

        def disconnect(self):
            self.disconnected += 1

    arm = Arm()

    class Session:
        feedback_timeout = .1

        def __init__(self, arm, gripper):
            pass

        def read_state(self, *, include_gripper):
            assert include_gripper is False
            return {}

    class Camera:
        sequence = 60
        actual_format = {'width': 160, 'height': 120, 'fps': 60., 'fourcc': 'MJPG'}

        def __init__(self, *unused):
            pass

        def read(self):
            return 50., image

        def close(self):
            pass

    class Feedback:
        def __init__(self, *unused):
            pass

        def snapshot(self):
            return [{'joint_monotonic_s': float(t), 'flange_monotonic_s': float(t),
                     'joint_rad': joints.tolist(), 'flange_xyz_rpy': [0.] * 6}
                    for t in np.linspace(48.8, 50., 121)]

        def close(self):
            pass

    class Detector:
        calls = 0

        def detect(self, frame):
            self.calls += 1
            # The first frame is missing a board: it must not mark the pose as saved.
            return {'pose_valid': self.calls > 1,
                    'charuco_corner_count': 24 if self.calls > 1 else 0,
                    'T_camera_board': np.eye(4).tolist(), 'reprojection_rms_px': .2}, image.copy()

    sdk = ModuleType('data_collect.cli.umi_piper_sdk')
    sdk.connect_arm = lambda channel: (arm, object(), {'software_version': 'fake'})
    sessions = ModuleType('data_collect.control.piper_sdk_session')
    sessions.PiperSdkSession = Session
    detector = Detector()
    modules = {'data_collect.cli.umi_piper_sdk': sdk,
               'data_collect.control.piper_sdk_session': sessions}
    with patch.dict('sys.modules', modules), \
            patch.object(module, 'LatestCamera', Camera), \
            patch.object(module, 'FeedbackReader', Feedback), \
            patch.object(module, 'make_detector', lambda *a: detector), \
            patch.object(module.cv2, 'imshow'), patch.object(module.cv2, 'destroyAllWindows'):
        module.record_or_preview(args)
        path = args.output / 'observations.jsonl'
        first = path.read_bytes()
        rows = [json.loads(line) for line in first.decode().splitlines()]
        assert len(rows) == 1 and rows[0]['capture_settings']['mode'] == 'automatic_joint_stability'
        assert rows[0]['sample_joint_rad'] == [0.] * 6
        assert rows[0]['charuco_corner_count'] == 24
        assert (args.output / 'pose_0001_raw.png').exists()
        # A restarted recorder must skip the same stationary pose without a Space key.
        args.once = False
        with patch.object(module.cv2, 'waitKey', side_effect=[-1, ord('q')]):
            module.record_or_preview(args)
        assert path.read_bytes() == first
        # A different pose is saved once; returning to the first is still skipped.
        joints[2] = np.deg2rad(5.)
        args.once = True
        module.record_or_preview(args)
        assert len(path.read_text().splitlines()) == 2
        second = path.read_bytes()
        joints[2] = 0.
        args.once = False
        with patch.object(module.cv2, 'waitKey', side_effect=[-1, ord('q')]):
            module.record_or_preview(args)
        assert path.read_bytes() == second
    assert arm.disconnected == 4


if __name__ == '__main__':
    import tempfile
    with tempfile.TemporaryDirectory() as folder:
        test_automatic_capture_and_resume(Path(folder))
    print('Piper automatic hand-eye recording check passed')
