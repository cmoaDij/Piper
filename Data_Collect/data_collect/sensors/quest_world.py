"""One read-only ADB input stream for the separate Quest world-space APK."""

import subprocess
import threading
import time
from decimal import Decimal

from data_collect.common.paths import ROOT
from data_collect.control.quest_world import LOG_TAG, PACKAGE, POSE_TIMEOUT_S, decode_packet


APK = ROOT / "vendor/quest_world/app/build/outputs/apk/debug/PiperQuestWorld-debug.apk"


def adb_serial(serial=None):
    result = subprocess.run(["adb", "devices"], check=True, capture_output=True, text=True, timeout=10)
    devices = [line.split()[0] for line in result.stdout.splitlines()[1:]
               if len(line.split()) >= 2 and line.split()[1] == "device"]
    if serial:
        if serial not in devices:
            raise RuntimeError(f"Quest ADB 设备不可用: {serial}")
        return serial
    if len(devices) != 1:
        raise RuntimeError("请连接一台 Quest 并允许 USB 调试；多设备时请指定 --serial")
    return devices[0]


class QuestWorldReader:
    def __init__(self, serial=None):
        self.serial = adb_serial(serial)
        self.lock = threading.Lock()
        self.packet = None
        self.error = None
        self.process = None
        self.thread = None
        installed = self.adb("shell", "pm", "path", PACKAGE).stdout
        if not installed.strip().startswith("package:"):
            raise RuntimeError("世界坐标 APK 尚未安装。执行 python -m data_collect.cli.quest_world --install")
        # Keep an already running app's LOCAL reference space intact. The
        # CLOCK_BOOTTIME barrier rejects old buffered packets without clearing
        # logcat or restarting the headset app for each read-only check.
        before = time.monotonic_ns()
        uptime = self.adb("shell", "cat", "/proc/uptime").stdout.split()[0]
        self.source_start_ns = int(Decimal(uptime) * 1_000_000_000)
        if time.monotonic_ns() - before >= POSE_TIMEOUT_S * 1e9:
            raise RuntimeError("Quest ADB 时钟采样耗时超过 500 ms，请检查 USB 连接后重试")
        self.device_offset_ns = before - self.source_start_ns
        # Android rejects -T 0 and silently substitutes 1. Explicitly use 1,
        # then reject any buffered packet older than our BOOTTIME barrier.
        self.process = subprocess.Popen(
            ["adb", "-s", self.serial, "logcat", "-T", "1", "-v", "raw", "-s", f"{LOG_TAG}:I", "*:S"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        self.thread = threading.Thread(target=self._read, name="quest-world-adb", daemon=True)
        self.thread.start()
        try:
            result = self.adb("shell", "am", "start", "-W", "-n", f"{PACKAGE}/.MainActivity")
        except (OSError, subprocess.SubprocessError):
            self.stop()
            raise
        if "Error" in result.stdout or "Error" in result.stderr:
            self.stop()
            raise RuntimeError("Quest 世界坐标应用启动失败: " + result.stdout + result.stderr)

    def adb(self, *args):
        return subprocess.run(["adb", "-s", self.serial, *args], check=True,
                              capture_output=True, text=True, timeout=30)

    def _read(self):
        try:
            for line in self.process.stdout:
                if not line.startswith('{"protocol":'):
                    continue
                packet = decode_packet(line)
                if packet["device_monotonic_ns"] < self.source_start_ns:
                    continue
                now = time.monotonic_ns()
                # Follow the minimum host-device offset after the BOOTTIME
                # barrier to detect queued old frames.
                offset = now - packet["device_monotonic_ns"]
                self.device_offset_ns = min(self.device_offset_ns, offset)
                packet["host_monotonic_ns"] = packet["device_monotonic_ns"] + self.device_offset_ns
                if now - packet["host_monotonic_ns"] > POSE_TIMEOUT_S * 1e9:
                    packet["valid"] = False
                with self.lock:
                    self.packet = packet
        except (ValueError, KeyError, TypeError, OSError) as error:
            self.error = f"Quest 世界坐标输入错误: {error}"
        finally:
            if self.error is None:
                self.error = "Quest 世界坐标 ADB 流已退出"

    def latest(self):
        with self.lock:
            return dict(self.packet) if self.packet is not None else None

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        if self.thread is not None:
            self.thread.join(timeout=2)

