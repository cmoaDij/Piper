#!/usr/bin/env bash
# Install the independent ROS/IK environment and build local packages; no devices start.
set -eo pipefail

piper_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$piper_project_dir"
mkdir -p .runtime/bin .runtime/wheels

if [[ ! -x .runtime/bin/micromamba ]]; then
    piper_mamba_archive="$(mktemp "$piper_project_dir/.runtime/micromamba.XXXXXX.tar.bz2")"
    curl --fail --location --retry 3 https://micro.mamba.pm/api/micromamba/linux-64/latest \
        --output "$piper_mamba_archive"
    tar -xjf "$piper_mamba_archive" -C .runtime bin/micromamba
    rm -- "$piper_mamba_archive"
fi

(
    # System Jazzy uses Python 3.12; keep it out of the Python 3.11 environment.
    unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    export MAMBA_ROOT_PREFIX="$piper_project_dir/.runtime/mamba"
    # Flat metadata also works with mirrors that do not serve conda shards reliably.
    export MAMBA_USE_SHARDED_REPODATA=false
    export MAMBA_LOCAL_REPODATA_TTL=3600
    if [[ ! -x .runtime/capture/bin/python ]]; then
        .runtime/bin/micromamba create -y --no-rc \
            --root-prefix "$MAMBA_ROOT_PREFIX" --prefix "$piper_project_dir/.runtime/capture" \
            -f environment.teleop.yml
    fi
    .runtime/bin/micromamba run --prefix "$piper_project_dir/.runtime/capture" \
        python -m pip install --find-links .runtime/wheels -c config/capture-constraints.txt \
        -e . ./vendor/pyagxarm pure-python-adb==0.3.0.dev0 colcon-notification
)

source "$piper_project_dir/env.sh" capture
(
    cd ros_ws
    colcon --log-base ../data/logs/build build --symlink-install \
        --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"
)
source "$piper_project_dir/env.sh" capture
python - <<'PY'
import rclpy
import pinocchio
from pinocchio import casadi
import agx_arm_msgs.msg
import agx_arm_ctrl.agx_arm_ctrl_single_node
import ppadb.client
import data_collect.application.collector
print("ROS / IK / SDK / ADB Python dependencies: OK")
PY
ros2 launch ./launch/collect.launch.py --show-args
echo "纯遥操环境安装完成；未启动机械臂、Quest、相机或触觉设备。"
echo "启动：./start.sh --terminal can_port:=can0 teleop_only:=true"
