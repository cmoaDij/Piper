#!/usr/bin/env python3
"""Compare UMI training TCP trajectories with a deployment target trajectory."""
import argparse
import json
from pathlib import Path
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation


def read_zarr_array(archive, prefix):
    meta = json.loads(archive.read(prefix + '/.zarray'))
    shape, chunks = meta['shape'], meta['chunks']
    dtype = np.dtype(meta['dtype'])
    output = np.empty(shape, dtype=dtype)
    count = (shape[0] + chunks[0] - 1) // chunks[0]
    for index in range(count):
        suffix = '.0' if len(shape) > 1 else ''
        raw = np.frombuffer(archive.read(f'{prefix}/{index}{suffix}'), dtype=dtype)
        rows = raw.size // int(np.prod(shape[1:]))
        values = raw.reshape((rows,) + tuple(shape[1:]))
        start = index * chunks[0]
        end = min(shape[0], start + rows)
        output[start:end] = values[:end - start]
    return output


def load_training(path):
    with zipfile.ZipFile(path) as archive:
        ends = read_zarr_array(archive, 'meta/episode_ends').astype(int)
        positions = read_zarr_array(archive, 'data/robot0_eef_pos')
        rotations = read_zarr_array(archive, 'data/robot0_eef_rot_axis_angle')
        widths = read_zarr_array(archive, 'data/robot0_gripper_width')
    episodes = []
    for index, end in enumerate(ends):
        start = 0 if index == 0 else int(ends[index - 1])
        position = positions[start:int(end)]
        rotation = rotations[start:int(end)]
        width = widths[start:int(end)].reshape(-1)
        start_rotation = Rotation.from_rotvec(rotation[0]).as_matrix()
        relative = (start_rotation.T @ (position - position[0]).T).T
        episodes.append({'tag_tcp': position, 'relative_tcp': relative, 'width_m': width})
    return episodes


def deployment_relative(input_dir):
    data_path = Path(input_dir) / 'trajectory_analysis' / 'trajectory_data.json'
    data = json.loads(data_path.read_text())
    actual = data['actual_feedback']
    expected = data['expected_selected']
    flange = np.asarray(actual[0]['flange'], dtype=float)
    start = np.eye(4)
    start[:3, :3] = Rotation.from_euler('xyz', flange[3:]).as_matrix()
    start[:3, 3] = np.asarray(flange[:3])
    tool = np.eye(4)
    tool[2, 3] = 0.195
    start_tcp = start @ tool
    targets = np.asarray([row['tcp'] for row in expected], dtype=float)
    relative = (start_tcp[:3, :3].T @ (targets - start_tcp[:3, 3]).T).T
    return data, relative


def render(training, deployment, output_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(15, 5.5), dpi=160)
    axes = [fig.add_subplot(131, projection='3d'), fig.add_subplot(132, projection='3d'),
            fig.add_subplot(133)]
    colors = plt.cm.Greys(np.linspace(.35, .85, len(training)))

    for episode, color in zip(training, colors):
        path = episode['tag_tcp']
        axes[0].plot(path[:, 0], path[:, 1], path[:, 2], color=color, linewidth=.8)
        relative = episode['relative_tcp']
        axes[1].plot(relative[:, 0], relative[:, 1], relative[:, 2], color=color, linewidth=.8)
    axes[0].set_title('Training TCP in tag frame')
    axes[1].set_title('Training TCP relative to episode start')
    for axis in axes[:2]:
        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')
        axis.set_zlabel('Z (m)')
        axis.set_box_aspect((1, 1, 1))

    axes[1].plot(deployment[:, 0], deployment[:, 1], deployment[:, 2],
                 color='#d62728', linewidth=2.0, label='real deployment target')
    axes[1].legend(loc='best')

    frame_count = min(30, max(len(row['relative_tcp']) for row in training))
    matrix = np.full((len(training), frame_count, 3), np.nan)
    for index, episode in enumerate(training):
        count = min(frame_count, len(episode['relative_tcp']))
        matrix[index, :count] = episode['relative_tcp'][:count]
    x = np.arange(frame_count)
    labels = ('X', 'Y', 'Z')
    for axis_index, label in enumerate(labels):
        axes[2].plot(x, np.nanmedian(matrix[:, :, axis_index], axis=0) * 1000,
                     linewidth=2, label=f'training median {label}')
        axes[2].fill_between(x,
                             np.nanpercentile(matrix[:, :, axis_index], 25, axis=0) * 1000,
                             np.nanpercentile(matrix[:, :, axis_index], 75, axis=0) * 1000,
                             alpha=.12)
        if len(deployment):
            count = min(frame_count, len(deployment))
            axes[2].plot(x[:count], deployment[:count, axis_index] * 1000,
                         '--', linewidth=1.4, label=f'real target {label}')
    axes[2].set_title('First 30 relative TCP samples')
    axes[2].set_xlabel('sample index')
    axes[2].set_ylabel('relative displacement (mm)')
    axes[2].grid(alpha=.25)
    axes[2].legend(fontsize=7, ncol=2)
    fig.tight_layout()
    figure_path = output_dir / 'training_vs_deployment_tcp.png'
    fig.savefig(figure_path)
    plt.close(fig)
    return figure_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--deployment-run', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    training = load_training(args.dataset)
    data, deployment = deployment_relative(args.deployment_run)
    figure = render(training, deployment, args.output_dir)

    first_count = min(30, max(len(row['relative_tcp']) for row in training))
    matrix = np.full((len(training), first_count, 3), np.nan)
    for i, episode in enumerate(training):
        count = min(first_count, len(episode['relative_tcp']))
        matrix[i, :count] = episode['relative_tcp'][:count]
    summary = {
        'dataset': str(args.dataset.resolve()),
        'deployment_run': str(args.deployment_run.resolve()),
        'training_episode_count': len(training),
        'training_frame_count': int(sum(len(row['relative_tcp']) for row in training)),
        'training_absolute_frame': 'table/reference tag frame, not Piper base frame',
        'training_relative_frame': 'each episode start TCP frame',
        'deployment_comparison_frame': 'first actual deployment TCP frame',
        'training_relative_median_first_30_mm': np.nanmedian(matrix, axis=0).tolist(),
        'training_relative_y_median_mm_at_samples': {
            str(i): float(np.nanmedian(matrix[:, i, 1]) * 1000)
            for i in (0, 5, 10, 20, 29) if i < first_count
        },
        'training_relative_y_positive_fraction_at_sample_10': float(np.mean(matrix[:, 10, 1] > 0)) if first_count > 10 else None,
        'deployment_relative_first_30_mm': (deployment[:30] * 1000).tolist(),
        'figure': str(figure.resolve()),
        'coordinate_note': 'Training absolute TCP uses tx_tag_tcp from the dataset-plan generator. The deployment target is converted from the policy relative TCP output into Piper base coordinates.'
    }
    (args.output_dir / 'training_vs_deployment_tcp.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({'figure': str(figure.resolve()), 'summary': str((args.output_dir / 'training_vs_deployment_tcp.json').resolve()),
                      'episodes': len(training), 'deployment_targets': len(deployment)}, indent=2))


if __name__ == '__main__':
    main()
