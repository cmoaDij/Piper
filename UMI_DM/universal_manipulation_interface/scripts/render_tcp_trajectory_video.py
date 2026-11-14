#!/usr/bin/env python3
"""Render an MP4 animation of expected and actual TCP paths in base coordinates."""
import argparse
import json
from pathlib import Path

import numpy as np


def read_data(input_dir):
    path = Path(input_dir) / 'trajectory_analysis' / 'trajectory_data.json'
    if not path.is_file():
        raise FileNotFoundError(f'Trajectory data not found: {path}')
    data = json.loads(path.read_text())
    expected = [row for row in data.get('expected_selected', [])
                if np.isfinite(row.get('time', np.nan)) and len(row.get('tcp', [])) == 3]
    actual = [row for row in data.get('actual_feedback', [])
              if np.isfinite(row.get('time', np.nan)) and len(row.get('tcp', [])) == 3]
    if not expected or not actual:
        raise ValueError('Both expected_selected and actual_feedback are required')
    expected.sort(key=lambda row: row['time'])
    actual.sort(key=lambda row: row['time'])
    return data, expected, actual


def limits(expected, actual):
    points = np.asarray([row['tcp'] for row in expected + actual], dtype=float)
    low, high = points.min(axis=0), points.max(axis=0)
    center = (low + high) / 2.0
    radius = max(float(np.max(high - low)) / 2.0, .05)
    return [(center[i] - radius, center[i] + radius) for i in range(3)]


def interpolate(rows, stamp):
    times = np.asarray([row['time'] for row in rows], dtype=float)
    values = np.asarray([row['tcp'] for row in rows], dtype=float)
    return np.asarray([np.interp(stamp, times, values[:, i]) for i in range(3)])


def render(input_dir, output, fps=30):
    data, expected, actual = read_data(input_dir)
    if fps <= 0:
        raise ValueError('fps must be positive')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter

    start = min(expected[0]['time'], actual[0]['time'])
    end = max(expected[-1]['time'], actual[-1]['time'])
    duration = max(0.1, end - start)
    frame_count = max(2, int(np.ceil(duration * fps)) + 1)
    frame_times = np.linspace(start, end, frame_count)
    axes_limits = limits(expected, actual)
    all_points = np.asarray([row['tcp'] for row in expected + actual], dtype=float)

    fig = plt.figure(figsize=(11, 8), dpi=140)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(*axes_limits[0])
    ax.set_ylim(*axes_limits[1])
    ax.set_zlim(*axes_limits[2])
    ax.set_xlabel('Base X (m)')
    ax.set_ylabel('Base Y (m)')
    ax.set_zlabel('Base Z (m)')
    ax.set_title('Piper TCP expected and actual trajectory')
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=24, azim=-58)

    base_radius = max(float(np.max(np.ptp(all_points, axis=0))) * .18, .03)
    origin = np.zeros(3)
    ax.quiver(*origin, base_radius, 0, 0, color='r', arrow_length_ratio=.12, linewidth=1.4)
    ax.quiver(*origin, 0, base_radius, 0, color='g', arrow_length_ratio=.12, linewidth=1.4)
    ax.quiver(*origin, 0, 0, base_radius, color='b', arrow_length_ratio=.12, linewidth=1.4)
    ax.text(base_radius, 0, 0, 'X', color='r')
    ax.text(0, base_radius, 0, 'Y', color='g')
    ax.text(0, 0, base_radius, 'Z', color='b')

    expected_line, = ax.plot([], [], [], color='#e07a00', linewidth=2.2,
                             label='Expected TCP')
    actual_line, = ax.plot([], [], [], color='#1769aa', linewidth=2.2,
                           label='Actual TCP')
    expected_point, = ax.plot([], [], [], 'o', color='#e07a00', markersize=7)
    actual_point, = ax.plot([], [], [], 'o', color='#1769aa', markersize=7)
    ax.legend(loc='upper left')
    info = ax.text2D(.02, .95, '', transform=ax.transAxes, va='top', family='monospace')

    expected_times = np.asarray([row['time'] for row in expected], dtype=float)
    expected_values = np.asarray([row['tcp'] for row in expected], dtype=float)
    actual_times = np.asarray([row['time'] for row in actual], dtype=float)
    actual_values = np.asarray([row['tcp'] for row in actual], dtype=float)

    def update(stamp):
        expected_mask = expected_times <= stamp
        actual_mask = actual_times <= stamp
        ep = expected_values[expected_mask]
        ap = actual_values[actual_mask]
        if len(ep) == 0:
            ep = expected_values[:1]
        if len(ap) == 0:
            ap = actual_values[:1]
        expected_line.set_data(ep[:, 0], ep[:, 1])
        expected_line.set_3d_properties(ep[:, 2])
        actual_line.set_data(ap[:, 0], ap[:, 1])
        actual_line.set_3d_properties(ap[:, 2])
        expected_current = interpolate(expected, stamp)
        actual_current = interpolate(actual, stamp)
        expected_point.set_data([expected_current[0]], [expected_current[1]])
        expected_point.set_3d_properties([expected_current[2]])
        actual_point.set_data([actual_current[0]], [actual_current[1]])
        actual_point.set_3d_properties([actual_current[2]])
        error_mm = float(np.linalg.norm(expected_current - actual_current) * 1000)
        info.set_text(
            f't = {stamp - start:6.2f} s\n'
            f'expected = [{expected_current[0]: .3f}, {expected_current[1]: .3f}, {expected_current[2]: .3f}] m\n'
            f'actual   = [{actual_current[0]: .3f}, {actual_current[1]: .3f}, {actual_current[2]: .3f}] m\n'
            f'error    = {error_mm:6.1f} mm')
        return expected_line, actual_line, expected_point, actual_point, info

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=fps, metadata={'title': 'Piper TCP trajectory', 'artist': 'UMI'})
    with writer.saving(fig, str(output), dpi=140):
        for stamp in frame_times:
            update(float(stamp))
            writer.grab_frame()
    plt.close(fig)
    return {'output': str(output.resolve()), 'fps': fps, 'duration_s': duration,
            'frame_count': frame_count, 'expected_points': len(expected),
            'actual_points': len(actual),
            'feedback_source': data.get('actual_feedback_source', 'unknown')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()
    output = args.output or args.input_dir / 'trajectory_analysis' / 'tcp_trajectory_base.mp4'
    print(json.dumps(render(args.input_dir, output, args.fps), indent=2))


if __name__ == '__main__':
    main()
