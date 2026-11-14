import json
import struct
import tempfile
from pathlib import Path

import numpy as np

from scripts.plot_deployment_trajectory import build_data, load_tool, export_csv
from scripts.analyze_deployment_run import aligned_errors


def test_reconstructs_selected_and_feedback_tcp():
    with tempfile.TemporaryDirectory(prefix='umi_trajectory_') as directory:
        root = Path(directory)
        (root / 'live_observation.jsonl').write_text(json.dumps({
            'prediction_id': 1,
            'predicted_flange_xyz_rpy': [[0, 0, 0, 0, 0, 0], [0.1, 0, 0, 0, 0, 0]],
            'latency_compensation': {
                'reference_scene_time_monotonic_s': 10.0,
                'action_period_s': 0.05,
            },
        }) + '\n')
        (root / 'control_events.jsonl').write_text(json.dumps({
            'event': 'sdk_calls_returned',
            'prediction_id': 1,
            'feedback_read_monotonic_s': 10.1,
            'feedback_flange_xyz_rpy': [0.01, 0, 0, 0, 0, 0],
            'actual_joint_rad': [0, 0, 0, 0, 0, 0],
            'feedback_gripper_m': 0.05,
            'predicted_flange_xyz_rpy': [0.1, 0, 0, 0, 0, 0],
            'selected_action_index': 1,
            'ik_diagnostic': {
                'planned_action_time_monotonic_s': 10.05,
                'ik_solution_joint_rad': [0.1, 0, 0, 0, 0, 0],
            },
            'candidate': {'gripper_m': 0.06},
        }) + '\n')
        tool, _ = load_tool(None)
        data = build_data(root, tool)
        assert len(data['prediction_horizons']) == 1
        assert len(data['expected_selected']) == 1
        assert len(data['actual_feedback']) == 1
        np.testing.assert_allclose(data['expected_selected'][0]['tcp'], [0.1, 0, 0.195])
        np.testing.assert_allclose(data['actual_feedback'][0]['tcp'], [0.01, 0, 0.195])


def test_continuous_feedback_and_independent_target_time_reconstruction():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        raw = []
        for base, x in ((10., 0), (10.1, 100000)):
            for i, values in enumerate(((x, 0), (0, 0), (0, 0))):
                raw.append({'can_id': 0x2A2 + i, 'host_recv_monotonic_s': base + i * .0001,
                            'payload_hex': struct.pack('>ii', *values).hex()})
            for i in range(3):
                raw.append({'can_id': 0x2A5 + i, 'host_recv_monotonic_s': base + .001 + i * .0001,
                            'payload_hex': struct.pack('>ii', 1000, -2000).hex()})
            raw.append({'can_id': 0x2A8, 'host_recv_monotonic_s': base + .002,
                        'payload_hex': struct.pack('>ihBB', 40000, 0, 64, 0).hex()})
        (root / 'can_feedback.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in raw))
        target = 10.0502
        candidate = {'planned_action_time_monotonic_s': target,
                     'arm_send_time_monotonic_s': target - .215,
                     'gripper_send_time_monotonic_s': target - .070,
                     'joints': [0.] * 6, 'gripper_m': .04, 'action_index': 9}
        common = {'prediction_id': 1, 'candidate': candidate,
                  'predicted_flange_xyz_rpy': [.05, 0, 0, 0, 0, 0],
                  'feedback_read_monotonic_s': 9.9,
                  'actual_flange_xyz_rpy': [9, 9, 9, 0, 0, 0]}
        events = [{**common, 'event': 'waypoint_scheduled'},
                  {**common, 'event': 'sdk_calls_returned', 'component': 'arm',
                   'planned_action_time_monotonic_s': target,
                   'scheduled_send_time_monotonic_s': target - .215,
                   'dispatch_time_monotonic_s': target - .215, 'dispatch_lateness_s': 0.},
                  {**common, 'event': 'gripper_sdk_calls_returned', 'component': 'gripper',
                   'scheduled_send_time_monotonic_s': target - .070,
                   'dispatch_time_monotonic_s': target - .070, 'dispatch_lateness_s': 0.}]
        (root / 'control_events.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in events))
        tool, _ = load_tool(None)
        data = build_data(root, tool)
        assert data['actual_feedback_source'] == 'continuous_raw_CAN'
        assert len(data['actual_feedback']) == 2 and len(data['joints_actual']) == 2
        assert data['actual_feedback'][0]['time'] != data['joints_actual'][0]['time']
        assert len(data['expected_selected']) == 1
        assert data['gripper_expected'][0]['time'] == target
        assert aligned_errors(data['expected_selected'], data['actual_feedback'], 'tcp')[0] < 1e-8
        assert aligned_errors([{'time': 11., 'tcp': [0., 0., 0.]}], data['actual_feedback'], 'tcp') == []
        export_csv(data, root)
        assert (root / 'expected_waypoints.csv').is_file() and (root / 'actual_tcp.csv').is_file()


if __name__ == '__main__':
    test_reconstructs_selected_and_feedback_tcp()
    test_continuous_feedback_and_independent_target_time_reconstruction()
