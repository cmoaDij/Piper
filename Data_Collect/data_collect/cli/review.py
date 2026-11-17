#!/usr/bin/env python3
"""Audit complete episodes, then open the local human-review service."""

import argparse
from pathlib import Path

from data_collect.common.config import load_config
from data_collect.web.review import serve_review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/collection.yaml")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    cfg = load_config(args.config)
    root = Path(cfg["data_root"])
    raw = root / "raw"
    from data_collect.dataset.audit import audit_batch

    print(f"扫描录制目录：{raw.resolve()}", flush=True)
    reports, manifest = audit_batch(raw, root / "audit", cfg["audit"])
    if reports:
        print(
            f"自动通过 {sum(r['status'] == 'PASS' for r in reports)}/{len(reports)}，"
            f"人工确认 {len(manifest['episodes'])} 条",
            flush=True,
        )
    else:
        print("未发现录制文件（episode_*.h5 / *.h5.partial）。机械臂能遥操不代表已录制；"
              "请检查采集终端是否出现 Recording 文件路径，或“无法开始录制”的原因。", flush=True)
    serve_review(root / "audit", cfg["audit"], port=args.port, raw_dir=raw)


if __name__ == "__main__":
    main()
