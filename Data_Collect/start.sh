#!/usr/bin/env bash
# One-command entry point. The capture environment and CAN must already be prepared.
set -e

piper_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$piper_project_dir"

if [[ "${1:-}" == --terminal ]]; then
    shift
    source "$piper_project_dir/env.sh" capture
    if [[ ! -f "$piper_project_dir/ros_ws/install/local_setup.bash" ]]; then
        echo "内部 ROS workspace 尚未构建；请先按 README 完成首次安装。" >&2
        exit 1
    fi
    adb start-server
    exec ros2 launch "$piper_project_dir/launch/collect.launch.py" "$@"
fi

piper_demo=false
for piper_argument in "$@"; do
    case "$piper_argument" in
        --demo) piper_demo=true ;;
        --help|-h)
            echo "用法：./start.sh [--can-port can0] [--port 8766] [--no-browser] [--demo]"
            echo "终端模式：./start.sh --terminal [can_port:=can0] [其他 ROS launch 参数]"
            echo "默认自动加载环境并启动工作台与遥操节点；按 A 从当前姿态对齐，不回零。"
            exit 0
            ;;
    esac
done

if [[ "$piper_demo" == true ]]; then
    exec python3 -m data_collect.cli.gui --start "$@"
fi

source "$piper_project_dir/env.sh" capture
if [[ ! -f "$piper_project_dir/ros_ws/install/local_setup.bash" ]]; then
    echo "内部 ROS workspace 尚未构建；请先按 README 完成首次安装。" >&2
    exit 1
fi
adb start-server
exec python -m data_collect.cli.gui --start "$@"
