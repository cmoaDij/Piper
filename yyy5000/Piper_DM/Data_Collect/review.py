#!/usr/bin/env python3
"""Audit complete episodes, then open the local human-review service."""

import argparse
from pathlib import Path

from data_collect.config import load_config
from data_collect.web_review import serve_review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/collection.yaml")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    cfg = load_config(args.config)
    root = Path(cfg["data_root"])
    raw = root / "raw"
    # An empty dataset can still open the page before HDF5 dependencies are installed.
    if raw.exists() and any(raw.glob("episode_*.h5*")):
        from data_collect.audit import audit_batch

        reports, manifest = audit_batch(raw, root / "audit", cfg["audit"])
        print(
            f"自动通过 {sum(r['status'] == 'PASS' for r in reports)}/{len(reports)}，"
            f"人工确认 {len(manifest['episodes'])} 条",
            flush=True,
        )
    serve_review(root / "audit", cfg["audit"], port=args.port)


if __name__ == "__main__":
    main()
