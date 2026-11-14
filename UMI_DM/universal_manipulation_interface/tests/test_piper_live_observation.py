import numpy as np
from scipy.spatial.transform import Rotation
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch

from scripts import piper_live_observation as live
from scripts.piper_live_observation import select_samples, tcp_pose


def test():
    frames = [(1.00, 'old'), (1.05, 'new')]
    samples = [(0.999, 'old-can'), (1.052, 'new-can')]
    selected, feedback = select_samples(frames, samples, 1.06)
    assert [entry[1] for entry in selected] == ['old', 'new']
    assert [entry[1] for entry in feedback] == ['old-can', 'new-can']
    for bad_frames, bad_samples, now in (
        (frames, samples, 2.0),
        ([(1.0, 'old'), (1.2, 'new')], samples, 1.2),
        (frames, [(0.0, 'wrong'), (0.1, 'wrong')], 1.06),
    ):
        try:
            select_samples(bad_frames, bad_samples, now)
        except ValueError:
            pass
        else:
            raise AssertionError('Stale or mismatched observations must be rejected')

    flange_tcp = np.eye(4)
    flange_tcp[2, 3] = 0.195
    pose = tcp_pose([1.0, 2.0, 3.0, 0.0, np.pi / 2, 0.0], flange_tcp)
    np.testing.assert_allclose(pose[:3], [1.195, 2.0, 3.0], atol=1e-8)
    np.testing.assert_allclose(Rotation.from_rotvec(pose[3:]).as_matrix(),
                               Rotation.from_euler('y', np.pi / 2).as_matrix())


def test_numeric_feedback_is_interpolated_at_camera_time():
    samples = [
        (1.00, np.array([0.0, 0.0]), 0.0, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
        (1.10, np.array([10.0, 20.0]), 10.0, np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0])),
    ]
    interpolated = live.interpolate_feedback(samples, 1.05, .025)
    np.testing.assert_allclose(interpolated[1], [5.0, 10.0])
    assert interpolated[2] == 5.0
    np.testing.assert_allclose(interpolated[3][:3], [.5, 1.0, 1.5])


def test_camera_and_robot_observation_delays_align_scene_times():
    frames = [(9.95, 'old'), (10.0, 'new')]
    samples = [(t, np.full(6, t), t, np.zeros(6)) for t in np.arange(9.75, 10.01, .005)]
    pair, feedback = select_samples(
        frames, samples, 10.01, camera_latency_s=.15, feedback_time_offset_s=.02)
    assert pair == tuple(frames)
    # Scene times 9.80/9.85 correspond to CAN receive times 9.82/9.87.
    np.testing.assert_allclose([item[0] for item in feedback], [9.82, 9.87])
    np.testing.assert_allclose(feedback[-1][1], np.full(6, 9.87))


def test_constant_gripper_upper_boundary_survives_interpolation():
    samples = [
        (1.0, np.zeros(6), 76.0, np.zeros(6)),
        (1.1, np.zeros(6), 76.0, np.zeros(6)),
    ]
    # Weighted addition previously produced 76.00000000000001 here, causing
    # a false rejection despite unchanged, in-range raw CAN feedback.
    for timestamp in np.linspace(1.0, 1.1, 1001):
        feedback = live.interpolate_feedback(samples, float(timestamp), .025)
        assert feedback[2] == 76.0
        width, clamped = live.physical_width_from_feedback(feedback[2] / 1000)
        assert width == .076 and not clamped
    try:
        live.physical_width_from_feedback(.0761)
    except ValueError:
        pass
    else:
        raise AssertionError('A genuinely excessive gripper width must still be rejected')


def test_transient_pair_is_skipped_and_next_frame_processed():
    class Done(Exception):
        pass

    class FakeCamera:
        def __init__(self, device):
            self.error = None
            self.calls = 0
            self.last_time = None

        def start(self):
            pass

        def recent(self):
            self.calls += 1
            self.last_time = time.monotonic()
            return [(self.last_time - d, 'image') for d in (.15, .10, .05, 0)]

        def close(self):
            pass

    camera = FakeCamera(None)

    class FakeCan:
        error = None

        def __init__(self, interface):
            pass

        def start(self):
            pass

        def recent(self):
            t = camera.last_time
            if camera.calls == 1:
                return [(t - .4, 'old'), (t - .2, 'old')]
            return [(t - .05, 'matched'), (t, 'matched')]

        def close(self):
            pass

    cfg = SimpleNamespace(task=SimpleNamespace(pose_repr=SimpleNamespace(
        obs_pose_repr='relative', action_pose_repr='relative')))
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / 'mask.json').write_text('{}')
        (root / 'setup.yaml').write_text('''
gripper:
  feedback_to_fingertip_offset_m: 0.0
  nominal_training_range_m: [0.0, 0.076]
flange_to_policy_tcp:
  rotation_verified: true
  rotation: [0.0, 0.0, 0.0]
  translation_m: [0.0, 0.0, 0.195]
''')
        argv = ['test', '--checkpoint', str(root / 'unused'), '--mask', str(root / 'mask.json'),
                '--setup', str(root / 'setup.yaml'), '--output', str(root), '--duration', '2']
        with patch.object(sys, 'argv', argv), patch.object(live.signal, 'signal'), \
             patch.object(live, 'CameraReader', return_value=camera), \
             patch.object(live, 'PassiveCan', FakeCan), \
             patch.object(live, 'load_policy', return_value=(cfg, object())), \
             patch.object(live, 'preprocess', side_effect=Done('next valid frame reached')):
            try:
                live.main()
            except Done:
                pass
            else:
                raise AssertionError('Next valid frame was not processed')
        skips = [json.loads(line) for line in (root / 'observation_skips.jsonl').read_text().splitlines()]
        assert len(skips) == 1
        assert skips[0]['reason'] == 'Camera and CAN receive times are too far apart'
        assert skips[0]['latest_can_age_ms'] >= 200
        assert max(skips[0]['camera_minus_nearest_can_ms']) > 25
        assert camera.calls >= 2
        assert (root / 'live_observation.jsonl').read_text() == ''


if __name__ == '__main__':
    test()
    test_numeric_feedback_is_interpolated_at_camera_time()
    test_camera_and_robot_observation_delays_align_scene_times()
    test_constant_gripper_upper_boundary_survives_interpolation()
    test_transient_pair_is_skipped_and_next_frame_processed()
    print('Piper live observation checks passed')
