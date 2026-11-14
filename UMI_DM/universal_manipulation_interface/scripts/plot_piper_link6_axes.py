#!/usr/bin/env python3
"""Plot official Piper-X flange/gripper STL in the link6 coordinate frame."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.spatial.transform import Rotation


def mesh(path, maximum=12000):
    record = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.fromfile(path, dtype=record, offset=84)['vertices'].astype(float)
    return triangles[::max(1, int(np.ceil(len(triangles) / maximum)))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    base_r = Rotation.from_euler('z', 90, degrees=True)
    origin = np.array([0., 0., .0045])
    finger_origin = np.array([0., 0., .138])
    fig = plt.figure(figsize=(10, 8), constrained_layout=True)
    ax = fig.add_subplot(projection='3d')
    for name, color in (('flange', '#7f8c8d'), ('gripper_base', '#aab4bd'),
                        ('gripper_link1', '#3c596b'), ('gripper_link2', '#3c596b')):
        points = mesh(args.mesh_dir / (name + '.stl'))
        if name == 'gripper_base':
            points = base_r.apply(points.reshape(-1, 3)).reshape(points.shape) + origin
        elif name.startswith('gripper_link'):
            # Official Xacro rotations and mimic joints at an example 40 mm opening.
            second = name.endswith('2')
            rotation = Rotation.from_euler('xyz', [90, 0, -180 if second else 0], degrees=True)
            points = points + [0, 0, .02]
            points = rotation.apply(points.reshape(-1, 3)).reshape(points.shape) + finger_origin
            points = base_r.apply(points.reshape(-1, 3)).reshape(points.shape) + origin
        ax.add_collection3d(Poly3DCollection(points * 1000, facecolors=color, edgecolors='none', alpha=.85))
    for axis, color, vector in (('X', '#d62728', [100, 0, 0]),
                                ('Y', '#2ca02c', [0, 100, 0]),
                                ('Z', '#1f77b4', [0, 0, 200])):
        ax.quiver(0, 0, 0, *vector, color=color, linewidth=3, arrow_length_ratio=.1)
        ax.text(*(np.asarray(vector) * 1.12), '+' + axis, color=color, fontsize=15, weight='bold')
    ax.scatter([0], [0], [0], color='black', s=25)
    ax.set(xlim=(-100, 120), ylim=(-120, 120), zlim=(-10, 220),
           xlabel='link6 X (mm)', ylabel='link6 Y (mm)', zlabel='link6 Z (mm)')
    ax.set_box_aspect((220, 240, 230))
    ax.view_init(elev=25, azim=-62)
    ax.set_title('Piper-X official model: link6 / flange axes', fontsize=15)
    ax.text2D(.02, .94, 'X: finger opening axis', transform=ax.transAxes, color='#d62728', fontsize=11)
    ax.text2D(.02, .90, 'Y: perpendicular to finger opening', transform=ax.transAxes, color='#2ca02c', fontsize=11)
    ax.text2D(.02, .86, 'Z: flange toward fingertips', transform=ax.transAxes, color='#1f77b4', fontsize=11)
    fig.supxlabel('Official model with stock gripper; actual GoPro axes and custom installation are not shown.', fontsize=10)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    print(args.output)


if __name__ == '__main__':
    main()
