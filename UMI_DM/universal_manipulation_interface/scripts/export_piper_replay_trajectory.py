#!/usr/bin/env python3
"""Export one Zarr episode as a small hardware-runtime replay NPZ."""

import argparse
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from piper_relative_replay import _latest_dataset, _load_episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=_latest_dataset())
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    pose, width, episode_count = _load_episode(args.input, args.episode)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        pose=pose,
        gripper_width=width,
        episode_index=np.asarray(args.episode, dtype=np.int64),
        episode_count=np.asarray(episode_count, dtype=np.int64),
    )
    print(f"Exported episode {args.episode}/{episode_count - 1} to {args.output}")


if __name__ == "__main__":
    main()
