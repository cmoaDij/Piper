import os
import subprocess
from typing import List, Optional

from .base_util import CanSystemInfoBase


class LinuxSocketCanSystemInfo(CanSystemInfoBase):
    """Helpers for querying Linux SocketCAN interface state."""

    @staticmethod
    def is_exists(channel: str) -> bool:
        """Return whether the CAN network device exists in sysfs."""
        return os.path.exists(f"/sys/class/net/{channel}")

    @staticmethod
    def is_up(channel: str) -> bool:
        """Return whether the CAN network device is administratively up."""
        oper_path = f"/sys/class/net/{channel}/operstate"
        if not os.path.exists(oper_path):
            return False
        with open(oper_path, "r") as f:
            state = f.read().strip()
        if state == "up":
            return True
        # Virtual CAN devices may report "unknown" even when the interface is up.
        if state == "unknown":
            flags_path = f"/sys/class/net/{channel}/flags"
            try:
                with open(flags_path, "r") as ff:
                    flags = int(ff.read().strip(), 16)
                return (flags & 0x1) != 0
            except (OSError, ValueError):
                return False
        return False

    @staticmethod
    def get_bitrate(channel: str) -> Optional[int]:
        """Return the configured bitrate parsed from `ip -details`, if available."""
        try:
            result = subprocess.run(
                ["ip", "-details", "link", "show", channel],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                check=False,
            )
            if result.returncode != 0:
                return None
            for line in result.stdout.split("\n"):
                if "bitrate" in line:
                    return int(line.split("bitrate ")[1].split(" ")[0])
        except (ValueError, IndexError, OSError):
            return None
        return None

    @staticmethod
    def get_available_can_channel() -> List[str]:
        """Return all CAN-like network interfaces available in sysfs."""
        can_ports = []
        for item in os.listdir("/sys/class/net/"):
            if "can" in item:
                can_ports.append(item)
        return can_ports

    @staticmethod
    def get_can_channel_info(channel: str) -> str:
        """Return a human-readable summary of the specified CAN interface."""
        try:
            with open(f"/sys/class/net/{channel}/operstate", "r") as file:
                state = file.read().strip()
            with open(f"/sys/class/net/{channel}/type", "r") as file:
                port_type = file.read().strip()
            return f"CAN port {channel}: State={state}, Type={port_type}"
        except FileNotFoundError:
            return f"CAN port {channel} not found."
