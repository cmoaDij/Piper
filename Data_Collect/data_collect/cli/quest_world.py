"""Install/check Quest world input without starting CAN, IK or recording."""

import argparse
import json
import subprocess
import time

import numpy as np

from data_collect.control.quest_world import WorldInputState
from data_collect.sensors.quest_world import APK, QuestWorldReader, adb_serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", action="store_true", help="安装独立世界坐标 APK，保留原有 APK")
    parser.add_argument("--check", action="store_true", help="只读核验实际 LOCAL 位姿；不启动机械臂")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--serial")
    args = parser.parse_args()
    if not 0 < args.duration <= 300:
        parser.error("duration 必须为 0–300 秒")
    try:
        serial = adb_serial(args.serial)
        if args.install:
            if not APK.is_file():
                raise RuntimeError(f"APK 尚未构建，请参考 docs/quest_world_usage.md: {APK}")
            subprocess.run(["adb", "-s", serial, "install", "-r", str(APK)], check=True, timeout=60)
            print("独立世界坐标程序已安装，原有相对坐标程序保留。", flush=True)
        if not args.check:
            if not args.install:
                parser.error("请选择 --install 或 --check")
            return
        reader = QuestWorldReader(serial)
        print("只读检查 LOCAL 输入；请戴上头显并轻移右手柄。不会连接 CAN 或下发机械臂命令。", flush=True)
        state = WorldInputState()
        frames, valid, tokens, positions = 0, 0, set(), []
        start = time.monotonic()
        last_seq = None
        try:
            while time.monotonic() - start < args.duration:
                if reader.error:
                    raise RuntimeError(reader.error)
                packet = reader.latest()
                if packet and (packet["session"], packet["seq"]) != last_seq:
                    state.update(packet)
                    last_seq = (packet["session"], packet["seq"])
                    frames += 1
                    tokens.add(state.token)
                    if state.valid():
                        valid += 1
                        positions.append(packet["grip"][:3])
                time.sleep(0.005)
        finally:
            reader.stop()
        result = {"space": "LOCAL", "frames": frames, "tracked_frames": valid,
                  "received_hz": round(frames / (time.monotonic() - start), 1),
                  "reference_generations": len(tokens),
                  "last_tracking": state.packet.get("tracking") if state.packet else None,
                  "position_range_m": np.ptp(positions, axis=0).round(4).tolist() if positions else None}
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if not valid:
            raise RuntimeError("尚未收到有效追踪位姿；请检查头显是否唤醒、应用处于前台且右手柄可见")
    except (RuntimeError, OSError, subprocess.SubprocessError, ValueError) as error:
        parser.exit(1, f"Quest 世界输入检查失败：{error}\n")


if __name__ == "__main__":
    main()
