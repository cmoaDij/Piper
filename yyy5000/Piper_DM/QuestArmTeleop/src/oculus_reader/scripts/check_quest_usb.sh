#!/usr/bin/env bash
# Verify that a Quest is available through ADB over USB.  This script does not
# start the robot or modify ADB's network-device configuration.
set -euo pipefail

if ! command -v adb >/dev/null 2>&1; then
    echo "adb was not found. Install Android platform-tools, then retry." >&2
    exit 1
fi

mapfile -t quest_usb_devices < <(
    adb devices | awk 'NR > 1 && $2 == "device" && $1 !~ /:/ { print $1 }'
)

if (( ${#quest_usb_devices[@]} == 0 )); then
    echo "No authorized USB ADB device was found." >&2
    echo "Connect the Quest with a data-capable USB cable, put it on, and accept the USB-debugging prompt." >&2
    echo "Current ADB devices:" >&2
    adb devices >&2
    exit 1
fi

if (( ${#quest_usb_devices[@]} > 1 )); then
    echo "More than one USB ADB device is connected; disconnect the extra device first." >&2
    printf '  %s\n' "${quest_usb_devices[@]}" >&2
    exit 1
fi

quest_serial="${quest_usb_devices[0]}"
adb -s "$quest_serial" wait-for-device

if ! adb -s "$quest_serial" shell pm path com.rail.oculus.teleop >/dev/null; then
    echo "Quest USB device '$quest_serial' is authorized, but com.rail.oculus.teleop is not installed." >&2
    echo "Install the teleop APK on this Quest, then run this check again." >&2
    exit 1
fi

echo "Quest USB ADB is ready: $quest_serial"
echo "Start wired teleoperation with:"
echo "  ros2 launch oculus_reader teleop_single_piper_usb.launch.py"
