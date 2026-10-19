#!/usr/bin/env bash
# Usage: source /absolute/path/Data_Collect/env.sh [capture|export]
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "请使用 source env.sh 或 source env.sh export" >&2
    exit 1
fi

_piper_activate_environment() {
    local root mode entry clean_path=""
    root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
    mode="${1:-capture}"
    if [[ "$mode" != capture && "$mode" != export ]]; then
        echo "用法：source env.sh [capture|export]" >&2
        return 1
    fi
    if [[ ! -x "$root/.runtime/$mode/bin/python" ]]; then
        echo "环境尚未安装：$root/.runtime/$mode" >&2
        return 1
    fi
    if declare -F deactivate >/dev/null; then deactivate; fi
    if declare -F micromamba >/dev/null && [[ -n "${CONDA_PREFIX:-}" ]]; then
        micromamba deactivate || return
    fi
    # System ROS is auto-sourced by this computer's .bashrc. Its Python 3.12
    # libraries must not enter the independent Python 3.11 capture environment.
    local -a entries
    IFS=: read -r -a entries <<< "$PATH"
    for entry in "${entries[@]}"; do
        case "$entry" in
            /opt/ros/*|"$root"/.runtime/capture/bin|"$root"/.runtime/export/bin|"$root"/ros_ws/install/*) ;;
            *) clean_path="${clean_path:+$clean_path:}$entry" ;;
        esac
    done
    export PATH="$clean_path"
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
    # setup.bash leaves these unexported in interactive shells. RoboStack's
    # setup.sh reuses AMENT_CURRENT_PREFIX, so a stale value reloads system ROS.
    unset AMENT_CURRENT_PREFIX AMENT_SHELL AMENT_PYTHON_EXECUTABLE COLCON_CURRENT_PREFIX
    unset LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
    unset ROS_AUTOMATIC_DISCOVERY_RANGE RMW_IMPLEMENTATION
    export PYTHONNOUSERSITE=1
    export UV_CACHE_DIR="$root/.runtime/uv-cache"
    export ROS_LOG_DIR="$root/data/logs"
    if [[ "$mode" == capture ]]; then
        # Keep numerical libraries from competing with camera/SDK/control threads.
        export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
        export MAMBA_ROOT_PREFIX="$root/.runtime/mamba"
        eval "$("$root/.runtime/bin/micromamba" shell hook --shell bash)" || return
        micromamba activate "$root/.runtime/capture" || return
        if [[ -f "$root/ros_ws/install/local_setup.bash" ]]; then
            source "$root/ros_ws/install/local_setup.bash" || return
        fi
        # Installed ROS scripts share the collection package's executor.
        export PYTHONPATH="$root${PYTHONPATH:+:$PYTHONPATH}"
        export PATH="$root/.runtime/platform-tools:$root/.runtime/bin:$PATH"
        export ADB_VENDOR_KEYS="$root/.runtime/android/adbkey"
    else
        source "$root/.runtime/export/bin/activate" || return
        export PATH="$root/.runtime/bin:$PATH"
    fi
    hash -r
    if [[ "$mode" == capture ]] && [[ "$(command -v ros2)" != "$root/.runtime/capture/bin/ros2" ]]; then
        echo "ROS 命令路径异常：$(command -v ros2)" >&2
        return 1
    fi
    echo "Piper 环境：$mode ($(python --version 2>&1))"
}

_piper_activate_environment "$@"
