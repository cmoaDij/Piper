"""Read-only camera detection; no video streams, ROS or CAN are opened."""

import argparse
import json

from data_collect.common.config import load_config
from data_collect.sensors.camera_selection import connected_gopro, connected_realsense, resolve_cameras


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/collection.yaml")
    args = parser.parse_args()
    rs, gopro = connected_realsense(), connected_gopro()
    print(json.dumps({"realsense_serials": sorted(rs), "gopro_capture_devices": gopro}, ensure_ascii=False, indent=2))
    cfg = load_config(args.config)
    try:
        resolve_cameras(cfg, realsense_serials=rs, gopro_devices=gopro)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, str(exc) + "\n")


if __name__ == "__main__":
    main()
