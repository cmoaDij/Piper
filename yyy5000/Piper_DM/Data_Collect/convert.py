#!/usr/bin/env python3
import argparse

from data_collect.config import inside_data_collect, local_path
from data_collect.export import convert, preflight


def main():
    parser = argparse.ArgumentParser(description="仅转换人工通过的 Episode 为标准 LeRobot v3")
    parser.add_argument("--manifest", default="data/audit/approved_episodes.json")
    parser.add_argument("--output", default="data/lerobot/piper_dataset")
    parser.add_argument("--repo-id", default="local/piper_dataset")
    parser.add_argument("--allow-simulated", action="store_true")
    parser.add_argument(
        "--check-only", action="store_true", help="只校验源文件/索引，不需要安装 LeRobot"
    )
    args = parser.parse_args()
    manifest = local_path(args.manifest)
    if args.check_only:
        _, entries, _ = preflight(manifest, args.allow_simulated)
        print(
            f"Preflight OK: {len(entries)} episodes, {sum(len(e['sync']['grid_host_ts_ns']) for e in entries)} frames"
        )
    else:
        print(
            convert(manifest, inside_data_collect(args.output), args.repo_id, args.allow_simulated)
        )


if __name__ == "__main__":
    main()
