#!/usr/bin/env python3
"""Create offline 3-D deployment trajectory plots.

The plot uses the model's selected TCP target and the Piper feedback received
by the SDK controller.  All model prediction horizons are also drawn faintly,
so a failed IK target remains visible instead of disappearing from the plot.
The feedback timestamp is the time at which the SDK read the robot state; it is
not an assertion that the robot had already reached the command.
"""
import argparse
import csv
import json
from pathlib import Path
import struct

import numpy as np
import yaml


def read_jsonl(path):
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text().splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def pose_matrix(pose):
    from scipy.spatial.transform import Rotation

    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError('Expected a finite xyz/rpy pose')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def tcp_position(pose, tool):
    return (pose_matrix(pose) @ tool)[:3, 3]


def load_tool(setup_path):
    tool = np.eye(4)
    if setup_path is None:
        tool[2, 3] = 0.195
        return tool, 'default 195 mm Z offset'
    setup = yaml.safe_load(Path(setup_path).read_text())
    config = setup.get('flange_to_policy_tcp', {})
    translation = np.asarray(config.get('translation_m', [0.0, 0.0, 0.195]), dtype=float)
    rotation = np.asarray(config.get('rotation', [0.0, 0.0, 0.0]), dtype=float)
    if translation.shape != (3,) or rotation.shape != (3,):
        raise ValueError('flange_to_policy_tcp must contain 3-element translation and rotation')
    from scipy.spatial.transform import Rotation

    tool[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    tool[:3, 3] = translation
    return tool, str(Path(setup_path).resolve())


def finite_pose(value):
    return isinstance(value, list) and len(value) == 6 and np.isfinite(np.asarray(value, dtype=float)).all()


def decode_can_feedback(rows, robot_observation_latency_s=0.0):
    """Reconstruct each feedback stream using its own actual receive times."""
    result = {'flange': [], 'joints': [], 'gripper': [], 'recording': None}
    joint_group, flange_group = {}, {}
    for row in rows:
        if row.get('event') == 'recording_summary':
            result['recording'] = row
        if 'can_id' not in row:
            continue
        cid = row['can_id']
        cid = int(cid, 0) if isinstance(cid, str) else int(cid)
        payload = bytes.fromhex(row['payload_hex'])
        stamp = float(row['host_recv_monotonic_s'])
        if len(payload) != 8:
            continue
        if cid in (0x2A5, 0x2A6, 0x2A7):
            joint_group[cid] = (stamp, struct.unpack('>ii', payload))
            if len(joint_group) == 3:
                stamps = [joint_group[k][0] for k in (0x2A5, 0x2A6, 0x2A7)]
                values = [v for k in (0x2A5, 0x2A6, 0x2A7) for v in joint_group[k][1]]
                result['joints'].append({'time': max(stamps) - robot_observation_latency_s,
                    'receive_time': max(stamps), 'group_span_s': max(stamps) - min(stamps),
                    'joint_rad': np.deg2rad(np.asarray(values) / 1000).tolist()})
                joint_group.clear()
        elif cid in (0x2A2, 0x2A3, 0x2A4):
            flange_group[cid] = (stamp, struct.unpack('>ii', payload))
            if len(flange_group) == 3:
                stamps = [flange_group[k][0] for k in (0x2A2, 0x2A3, 0x2A4)]
                values = [v for k in (0x2A2, 0x2A3, 0x2A4) for v in flange_group[k][1]]
                pose = [v * 1e-6 for v in values[:3]] + np.deg2rad(np.asarray(values[3:]) / 1000).tolist()
                result['flange'].append({'time': max(stamps) - robot_observation_latency_s,
                    'receive_time': max(stamps), 'group_span_s': max(stamps) - min(stamps), 'flange': pose})
                flange_group.clear()
        elif cid == 0x2A8 and payload[7] == 0:
            width = struct.unpack('>i', payload[:4])[0] / 1e6
            result['gripper'].append({'time': stamp, 'receive_time': stamp,
                'width_m': max(0., width), 'raw_width_m': width,
                'force_n': struct.unpack('>h', payload[4:6])[0] / 1000, 'status': payload[6]})
    return result


def build_data(input_dir, tool, robot_observation_latency_s=0.0):
    predictions = read_jsonl(input_dir / 'live_observation.jsonl')
    controls = read_jsonl(input_dir / 'control_events.jsonl')
    expected = []
    actual = []
    horizons = []
    joints_expected = []
    joints_actual = []
    gripper_expected = []
    gripper_actual = []
    scheduled, dispatches, rejected = [], [], []

    for row in predictions:
        poses = row.get('predicted_flange_xyz_rpy')
        timing = row.get('latency_compensation') or {}
        reference = timing.get('reference_scene_time_monotonic_s')
        period = timing.get('action_period_s')
        if isinstance(poses, list) and reference is not None and period is not None:
            points = []
            times = []
            for index, pose in enumerate(poses):
                if finite_pose(pose):
                    points.append(tcp_position(pose, tool).tolist())
                    times.append(float(reference) + index * float(period))
            if points:
                horizons.append({'prediction_id': row.get('prediction_id'), 'times': times, 'tcp': points,
                                 'gripper_width_m': row.get('predicted_physical_width_m')})

    for row in controls:
        stamp = row.get('feedback_read_monotonic_s')
        feedback = row.get('actual_flange_xyz_rpy') or row.get('feedback_flange_xyz_rpy')
        if stamp is not None and finite_pose(feedback):
            actual.append({'time': float(stamp), 'tcp': tcp_position(feedback, tool).tolist(),
                           'flange': feedback, 'joint_rad': row.get('actual_joint_rad'),
                           'gripper_m': row.get('feedback_gripper_m'),
                           'prediction_id': row.get('prediction_id'),
                           'event': row.get('event')})
        diagnostic = row.get('ik_diagnostic') or {}
        predicted = row.get('predicted_flange_xyz_rpy') or diagnostic.get('predicted_flange_xyz_rpy')
        planned = diagnostic.get('planned_action_time_monotonic_s')
        candidate = row.get('candidate') or {}
        planned = candidate.get('planned_action_time_monotonic_s', planned)
        event = row.get('event')
        component = row.get('component')
        if event == 'waypoint_scheduled':
            scheduled.append({'prediction_id': row.get('prediction_id'),
                'action_index': candidate.get('action_index'),
                'time': planned, 'arm_send_time': candidate.get('arm_send_time_monotonic_s'),
                'gripper_send_time': candidate.get('gripper_send_time_monotonic_s'),
                'joint_rad': candidate.get('joints'), 'gripper_m': candidate.get('gripper_m'),
                'tcp': tcp_position(predicted, tool).tolist() if finite_pose(predicted) else None})
        if event == 'prepare_failed' and planned is not None and finite_pose(predicted):
            rejected.append({'time': float(planned), 'tcp': tcp_position(predicted, tool).tolist(),
                             'prediction_id': row.get('prediction_id'), 'error': row.get('error')})
        sent_arm = event in ('sdk_calls_returned', 'observe_candidate') and component in (None, 'arm')
        sent_gripper = event in ('gripper_sdk_calls_returned', 'observe_gripper_candidate') or (sent_arm and component is None)
        if sent_arm and planned is not None and finite_pose(predicted):
            expected.append({'time': float(planned), 'tcp': tcp_position(predicted, tool).tolist(),
                             'flange': predicted, 'prediction_id': row.get('prediction_id'),
                             'action_index': row.get('selected_action_index'),
                             'event': row.get('event')})
            q = candidate.get('joints', diagnostic.get('ik_solution_joint_rad'))
            if isinstance(q, list) and len(q) == 6 and np.isfinite(np.asarray(q, dtype=float)).all():
                joints_expected.append({'time': float(planned), 'joint_rad': q,
                                        'prediction_id': row.get('prediction_id')})
        q_actual = row.get('actual_joint_rad')
        if stamp is not None and isinstance(q_actual, list) and len(q_actual) == 6:
            if np.isfinite(np.asarray(q_actual, dtype=float)).all():
                joints_actual.append({'time': float(stamp), 'joint_rad': q_actual})
        width = candidate.get('gripper_m')
        if sent_gripper and planned is not None and width is not None and np.isfinite(width):
            gripper_expected.append({'time': float(planned), 'width_m': float(width),
                                    'prediction_id': row.get('prediction_id')})
        if component in ('arm', 'gripper') and event in (
                'sdk_calls_returned', 'gripper_sdk_calls_returned', 'observe_candidate', 'observe_gripper_candidate'):
            dispatches.append({'component': component, 'prediction_id': row.get('prediction_id'),
                'target_time': planned, 'scheduled_send_time': row.get('scheduled_send_time_monotonic_s'),
                'dispatch_time': row.get('dispatch_time_monotonic_s'),
                'lateness_s': row.get('dispatch_lateness_s')})
        feedback_width = row.get('feedback_gripper_m')
        if stamp is not None and feedback_width is not None and np.isfinite(feedback_width):
            gripper_actual.append({'time': float(stamp), 'width_m': float(feedback_width)})

    raw = decode_can_feedback(read_jsonl(input_dir / 'can_feedback.jsonl'), robot_observation_latency_s)
    if raw['flange']:
        actual = [{**row, 'tcp': tcp_position(row['flange'], tool).tolist()} for row in raw['flange']]
    if raw['joints']:
        joints_actual = raw['joints']
    if raw['gripper']:
        gripper_actual = raw['gripper']

    expected.sort(key=lambda x: x['time'])
    actual.sort(key=lambda x: x['time'])
    horizons.sort(key=lambda x: x['times'][0] if x['times'] else float('inf'))
    joints_expected.sort(key=lambda x: x['time'])
    joints_actual.sort(key=lambda x: x['time'])
    gripper_actual.sort(key=lambda x: x['time'])
    gripper_expected.sort(key=lambda x: x['time'])
    return {
        'expected_selected': expected,
        'actual_feedback': actual,
        'prediction_horizons': horizons,
        'joints_expected': joints_expected,
        'joints_actual': joints_actual,
        'gripper_expected': gripper_expected,
        'gripper_actual': gripper_actual,
        'scheduled_waypoints': scheduled, 'component_dispatches': dispatches,
        'rejected_selected': rejected, 'feedback_recording': raw['recording'],
        'actual_feedback_source': 'continuous_raw_CAN' if raw['flange'] else 'SDK_event_snapshots',
        'robot_observation_latency_s': robot_observation_latency_s,
        'timestamp_note': 'Actual times are host receive times corrected only by the configured observation delay; unknown delay is approximated as zero.',
    }


def export_csv(data, output_dir):
    def write(name, header, rows):
        with (output_dir / name).open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)

    write('expected_waypoints.csv', ['target_time_monotonic_s', 'arm_send_time_monotonic_s',
        'gripper_send_time_monotonic_s', 'prediction_id', 'action_index', 'tcp_x_m', 'tcp_y_m', 'tcp_z_m',
        *[f'joint_{i}_rad' for i in range(1, 7)], 'gripper_m'],
        [[r['time'], r['arm_send_time'], r['gripper_send_time'], r['prediction_id'], r['action_index'],
          *(r['tcp'] or [None] * 3), *(r['joint_rad'] or [None] * 6), r['gripper_m']]
         for r in data['scheduled_waypoints']])
    write('actual_tcp.csv', ['time_monotonic_s', 'tcp_x_m', 'tcp_y_m', 'tcp_z_m',
        'flange_x_m', 'flange_y_m', 'flange_z_m', 'flange_rx_rad', 'flange_ry_rad', 'flange_rz_rad'],
        [[r['time'], *r['tcp'], *r['flange']] for r in data['actual_feedback']])
    write('actual_joints.csv', ['time_monotonic_s', *[f'joint_{i}_rad' for i in range(1, 7)]],
          [[r['time'], *r['joint_rad']] for r in data['joints_actual']])
    write('actual_gripper.csv', ['time_monotonic_s', 'width_m', 'raw_width_m'],
          [[r['time'], r['width_m'], r.get('raw_width_m')] for r in data['gripper_actual']])
    write('component_dispatches.csv', ['component', 'prediction_id', 'target_time_monotonic_s',
        'scheduled_send_time_monotonic_s', 'actual_send_time_monotonic_s', 'lateness_s'],
        [[r['component'], r['prediction_id'], r['target_time'], r['scheduled_send_time'],
          r['dispatch_time'], r['lateness_s']] for r in data['component_dispatches']])


def relative_time(values, origin):
    return np.asarray(values, dtype=float) - float(origin)


def set_equal_axes(ax, points):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return
    low = np.min(points, axis=0)
    high = np.max(points, axis=0)
    center = (low + high) / 2.0
    radius = max(float(np.max(high - low)) / 2.0, 0.01)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def plot_trajectory(data, output_dir, title, show=False):
    import matplotlib

    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    expected = data['expected_selected']
    actual = data['actual_feedback']
    horizons = data['prediction_horizons']
    all_points = [x['tcp'] for x in expected + actual]
    all_points.extend(x['tcp'] for x in data['rejected_selected'])
    all_points.extend(point for horizon in horizons for point in horizon['tcp'])
    if not all_points:
        raise ValueError('No trajectory points were found in the deployment logs')
    origin = min([x['time'] for x in expected + actual] +
                 [h['times'][0] for h in horizons if h['times']])

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    for horizon in horizons:
        points = np.asarray(horizon['tcp'], dtype=float)
        if len(points) >= 2:
            ax.plot(points[:, 0], points[:, 1], points[:, 2], color='#f2a65a',
                    alpha=0.12, linewidth=0.8)
    if expected:
        points = np.asarray([x['tcp'] for x in expected], dtype=float)
        ax.plot(points[:, 0], points[:, 1], points[:, 2], color='#e07a00',
                linewidth=2.2, label='expected TCP (selected waypoints)')
        ax.scatter(points[0, 0], points[0, 1], points[0, 2], color='#e07a00', marker='o', s=45)
        ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], color='#e07a00', marker='x', s=60)
    if actual:
        points = np.asarray([x['tcp'] for x in actual], dtype=float)
        ax.plot(points[:, 0], points[:, 1], points[:, 2], color='#1769aa',
                linewidth=2.2, label='actual TCP (CAN receive feedback)')
        ax.scatter(points[0, 0], points[0, 1], points[0, 2], color='#1769aa', marker='o', s=45)
        ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], color='#1769aa', marker='x', s=60)
    if data['rejected_selected']:
        points = np.asarray([row['tcp'] for row in data['rejected_selected']])
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='#c62828', marker='x', s=50,
                   label='rejected targets (not sent)')
    set_equal_axes(ax, all_points)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title + '\nFaint lines: complete prediction horizons')
    ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(output_dir / 'trajectory_3d.png', dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    labels = ('X', 'Y', 'Z')
    if expected:
        ep = np.asarray([x['tcp'] for x in expected])
        et = relative_time([x['time'] for x in expected], origin)
        for i, axis in enumerate(axes):
                axis.plot(et, ep[:, i], color='#e07a00', label='expected')
    if actual:
        ap = np.asarray([x['tcp'] for x in actual])
        at = relative_time([x['time'] for x in actual], origin)
        for i, axis in enumerate(axes):
            axis.plot(at, ap[:, i], color='#1769aa', label='feedback')
    for axis, label in zip(axes, labels):
        axis.set_ylabel(f'{label} (m)')
        axis.grid(alpha=0.25)
        axis.legend(loc='best')
    axes[-1].set_xlabel('relative time (s)')
    fig.suptitle(title + ': TCP XYZ over time')
    fig.tight_layout()
    fig.savefig(output_dir / 'tcp_xyz_vs_time.png', dpi=180)
    plt.close(fig)

    if data['joints_expected'] or data['joints_actual']:
        fig, axes = plt.subplots(6, 1, figsize=(12, 13), sharex=True)
        if data['joints_expected']:
            jt = relative_time([x['time'] for x in data['joints_expected']], origin)
            jq = np.rad2deg(np.asarray([x['joint_rad'] for x in data['joints_expected']]))
            for i, axis in enumerate(axes):
                axis.plot(jt, jq[:, i], color='#e07a00', label='expected IK')
        if data['joints_actual']:
            jt = relative_time([x['time'] for x in data['joints_actual']], origin)
            jq = np.rad2deg(np.asarray([x['joint_rad'] for x in data['joints_actual']]))
            for i, axis in enumerate(axes):
                axis.plot(jt, jq[:, i], color='#1769aa', label='feedback')
        for i, axis in enumerate(axes):
            axis.set_ylabel(f'J{i + 1} (deg)')
            axis.grid(alpha=0.25)
            axis.legend(loc='best')
        axes[-1].set_xlabel('relative time (s)')
        fig.suptitle(title + ': joint targets and feedback')
        fig.tight_layout()
        fig.savefig(output_dir / 'joints_expected_vs_feedback.png', dpi=180)
        plt.close(fig)

    if data['gripper_expected'] or data['gripper_actual']:
        fig, axis = plt.subplots(figsize=(12, 4))
        if data['gripper_expected']:
            axis.plot(relative_time([x['time'] for x in data['gripper_expected']], origin),
                      [1000 * x['width_m'] for x in data['gripper_expected']],
                      color='#e07a00', label='expected')
        if data['gripper_actual']:
            axis.plot(relative_time([x['time'] for x in data['gripper_actual']], origin),
                      [1000 * x['width_m'] for x in data['gripper_actual']],
                      color='#1769aa', label='feedback')
        axis.set_xlabel('relative time (s)')
        axis.set_ylabel('gripper width (mm)')
        axis.grid(alpha=0.25)
        axis.legend(loc='best')
        fig.tight_layout()
        fig.savefig(output_dir / 'gripper_expected_vs_feedback.png', dpi=180)
        plt.close(fig)

    if data['component_dispatches']:
        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        for component, color in (('arm', '#e07a00'), ('gripper', '#207a4b')):
            rows = [r for r in data['component_dispatches'] if r['component'] == component
                    and r['dispatch_time'] is not None and r['target_time'] is not None]
            if not rows:
                continue
            t = relative_time([r['target_time'] for r in rows], origin)
            axes[0].plot(t, [1000 * (r['target_time'] - r['scheduled_send_time']) for r in rows],
                         '--', color=color, label=f'{component} planned lead')
            axes[0].plot(t, [1000 * (r['target_time'] - r['dispatch_time']) for r in rows],
                         '.', color=color, label=f'{component} actual send lead')
            axes[1].plot(t, [1000 * r['lateness_s'] for r in rows], '.', color=color, label=component)
        axes[0].set_ylabel('lead before target time (ms)')
        axes[1].set_ylabel('dispatch lateness (ms)')
        axes[1].set_xlabel('target relative time (s)')
        for axis in axes:
            axis.grid(alpha=.25)
            axis.legend(loc='best')
        fig.tight_layout()
        fig.savefig(output_dir / 'dispatch_timing.png', dpi=180)
        plt.close(fig)

    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True,
                        help='单次部署输出目录，例如 deploy/.../runs/sdk_...')
    parser.add_argument('--output-dir', type=Path,
                        help='默认写入 input-dir/trajectory_analysis')
    parser.add_argument('--setup', type=Path,
                        help='piper_policy_setup.yaml，用于读取 TCP 偏移')
    parser.add_argument('--show', action='store_true', help='交互式显示图形')
    args = parser.parse_args()
    if not args.input_dir.is_dir():
        parser.error(f'input directory does not exist: {args.input_dir}')
    output_dir = args.output_dir or args.input_dir / 'trajectory_analysis'
    output_dir.mkdir(parents=True, exist_ok=True)
    setup_path = args.setup or (args.input_dir / 'setup.yaml' if (args.input_dir / 'setup.yaml').is_file() else None)
    tool, tool_source = load_tool(setup_path)
    setup = yaml.safe_load(Path(setup_path).read_text()) if setup_path is not None else {}
    robot_observation_delay = float(setup.get('latency_compensation', {}).get('robot_observation_latency_s', 0.0))
    data = build_data(args.input_dir, tool, robot_observation_delay)
    title = f'Piper deployment trajectory: {args.input_dir.name}'
    export = {'input_dir': str(args.input_dir.resolve()), 'tool_source': tool_source,
              'tool_matrix': tool.tolist(), **data}
    (output_dir / 'trajectory_data.json').write_text(json.dumps(export, indent=2) + '\n')
    export_csv(data, output_dir)
    plot_trajectory(data, output_dir, title, show=args.show)
    print(json.dumps({'output_dir': str(output_dir.resolve()),
                      'expected_points': len(data['expected_selected']),
                      'actual_feedback_points': len(data['actual_feedback']),
                      'prediction_horizons': len(data['prediction_horizons']),
                      'feedback_source': data['actual_feedback_source'],
                      'gripper_feedback_points': len(data['gripper_actual']),
                      'component_dispatches': len(data['component_dispatches'])}, indent=2))


if __name__ == '__main__':
    main()
