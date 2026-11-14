#!/usr/bin/env bash
# The SDK and CUDA model run in their existing Python 3.11/3.9 environments.
set -eo pipefail

usage() {
  printf '用法：%s [--align|--home|--execute|--observe|--check] [秒数] [选项]\n' "$0"
  printf '  --align    只开相机和训练首帧叠加，不加载模型、不连接 CAN\n'
  printf '  --home     六个关节回零，完成后退出，不改变编码器零点或夹爪开度\n'
  printf '  --execute  从当前姿态启动 GoPro 模型控制；默认持续运行，Ctrl+C 结束\n'
  printf '  --observe  运行推理与目标检查，不使能或移动机械臂\n'
  printf '  --session PATH       session 根目录\n'
  printf '  --checkpoint PATH    明确指定模型 checkpoint\n'
  printf '  --setup PATH         Piper 部署配置\n'
  printf '  --mask PATH          训练遮罩\n'
  printf '  --output-root PATH   部署日志根目录\n'
  printf '  --dashboard-port N   本地 Dashboard 端口（默认 8768）\n'
  printf '  --disable-inference-augmentations  关闭推理阶段 RandomCrop 和 ColorJitter\n'
  printf '  --record             记录原始 GoPro 帧和完整模型输入\n'
  printf '  --record-dir PATH    指定记录目录，同时启用记录\n'
  printf '  --match-dataset PATH 可选：训练 Zarr 数据集，启用训练首帧叠加\n'
  printf '  --match-episode N    初始参考片段编号，从 0 开始（页面可切换）\n'
  printf '  --match-alpha VALUE  训练图像透明度 0--1，默认 0.5（页面可调整）\n'
  printf '  秒数为 0 表示持续运行\n'
}

umi=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(dirname -- "$umi")
piper=$(dirname -- "$workspace")/Piper/Data_Collect
model_python=/home/l1/miniconda3/envs/umi/bin/python

mode=--check
duration=0
session=${UMI_SESSION:-$workspace/my_session2}
[[ -d "$session" ]] || session=$workspace/my_session
setup=${UMI_SETUP:-$workspace/deploy/current/piper_policy_setup.yaml}
checkpoint=${UMI_CHECKPOINT:-}
checkpoint_explicit=0
[[ -n "$checkpoint" ]] && checkpoint_explicit=1
mask=${UMI_MASK:-$session/calibration/training_mask.json}
output_root=${UMI_OUTPUT_ROOT:-$workspace/deploy/current/runs}
dashboard_port=${PIPER_PREVIEW_PORT:-8768}
disable_inference_augmentations=0
record_enabled=0
record_dir=''
match_dataset=''
match_episode=0
match_alpha=0.5

if [[ -z "$checkpoint" ]]; then
  if [[ -f "$workspace/train/current/checkpoints/latest.ckpt" ]]; then
    checkpoint=$workspace/train/current/checkpoints/latest.ckpt
  else
    checkpoint=$session/training_49/model/checkpoints/latest.ckpt
  fi
fi
if [[ ! -f "$setup" && -f "$workspace/my_session/deployment_check/piper_policy_setup.yaml" ]]; then
  setup=$workspace/my_session/deployment_check/piper_policy_setup.yaml
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --check|--home|--observe|--execute|--align)
      mode=$1; shift
      if [[ $# -gt 0 && "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]; then duration=$1; shift; fi ;;
    --session) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; session=$2; shift 2 ;;
    --checkpoint) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; checkpoint=$2; checkpoint_explicit=1; shift 2 ;;
    --setup) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; setup=$2; shift 2 ;;
    --mask) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; mask=$2; shift 2 ;;
    --output-root) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; output_root=$2; shift 2 ;;
    --dashboard-port) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; dashboard_port=$2; shift 2 ;;
    --disable-inference-augmentations) disable_inference_augmentations=1; shift ;;
    --record) record_enabled=1; shift ;;
    --record-dir) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; record_enabled=1; record_dir=$2; shift 2 ;;
    --match-dataset) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; match_dataset=$2; shift 2 ;;
    --match-episode) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; match_episode=$2; shift 2 ;;
    --match-alpha) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; match_alpha=$2; shift 2 ;;
    *)
      if [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]; then duration=$1; shift
      else printf '未知参数：%s\n' "$1" >&2; usage >&2; exit 2; fi ;;
  esac
done
if [[ "$checkpoint_explicit" == 0 ]]; then
  if [[ -f "$workspace/train/current/checkpoints/latest.ckpt" ]]; then
    checkpoint=$workspace/train/current/checkpoints/latest.ckpt
  else
    checkpoint=$session/training_49/model/checkpoints/latest.ckpt
  fi
fi
if [[ ! -f "$setup" && -f "$workspace/my_session/deployment_check/piper_policy_setup.yaml" ]]; then
  setup=$workspace/my_session/deployment_check/piper_policy_setup.yaml
fi
mkdir -p "$output_root"
session=$(readlink -f "$session")
setup=$(readlink -f "$setup")
mask=$(readlink -f "$mask")
if [[ "$mode" != --align ]]; then checkpoint=$(readlink -f "$checkpoint"); fi
output_root=$(readlink -f "$output_root")
if [[ ! "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf '秒数必须为非负数，0 表示持续运行。\n' >&2
  exit 2
fi

if [[ "$mode" == --check || "$mode" == --home ]]; then
  cd "$piper"
  source ./env.sh capture
  exec python -m data_collect.cli.umi_piper_sdk --setup "$setup" "$mode"
fi

if [[ "$mode" == --align && -z "$match_dataset" ]]; then
  printf '%s\n' '--align 需要通过 --match-dataset 指定训练数据集。' >&2
  exit 2
fi
if [[ -n "$match_dataset" ]]; then
  if [[ ! -e "$match_dataset" ]]; then
    printf '训练数据集不存在：%s\n' "$match_dataset" >&2; exit 2
  fi
  match_dataset=$(readlink -f "$match_dataset")
fi
inputs=("$setup" "$mask")
if [[ "$mode" != --align ]]; then inputs+=("$checkpoint"); fi
for input in "${inputs[@]}"; do
  if [[ ! -f "$input" ]]; then
    printf '缺少运行文件：%s\n' "$input" >&2
    exit 2
  fi
done
if [[ ! -x "$model_python" ]]; then
  printf '模型 Python 不存在：%s\n' "$model_python" >&2
  exit 2
fi

mkdir -p "$output_root"
output=$(mktemp -d "$output_root/sdk_$(date +%Y%m%d_%H%M%S)_XXXXXX")
cp -- "$setup" "$output/setup.yaml"
runtime_setup=$output/setup.yaml
if [[ "$record_enabled" == 1 && -z "$record_dir" ]]; then
  record_dir=$output/recording
fi
if [[ "$record_enabled" == 1 ]]; then
  mkdir -p "$record_dir"
fi
cat > "$output/manifest.yaml" <<EOF
mode: $mode
duration_s: $duration
session: $session
checkpoint: $checkpoint
setup: $setup
setup_snapshot: $runtime_setup
mask: $mask
dashboard_port: $dashboard_port
disable_inference_augmentations: $disable_inference_augmentations
record_enabled: $record_enabled
record_dir: ${record_dir:-}
match_dataset: ${match_dataset:-}
match_episode: $match_episode
match_alpha: $match_alpha
started_at: $(date --iso-8601=seconds)
EOF
match_args=()
if [[ -n "$match_dataset" ]]; then
  match_args=(--match-dataset "$match_dataset" --match-episode "$match_episode" --match-alpha "$match_alpha")
fi
if [[ "$mode" == --align ]]; then
  align_args=(--setup "$runtime_setup" --mask "$mask" --output "$output"
    --preview-port "$dashboard_port" --duration "$duration" "${match_args[@]}")
  if [[ "$record_enabled" == 1 ]]; then align_args+=(--record-dir "$record_dir"); fi
  cd "$umi"
  printf '相机起始场景对齐，输出目录：%s\n' "$output"
  exec env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 \
    "$model_python" scripts/piper_camera_alignment.py "${align_args[@]}" \
    > >(tee -a "$output/alignment.log") 2>&1
fi
socket_dir=$(mktemp -d /tmp/umi_piper_sdk_XXXXXXXX)
socket_path=$socket_dir/prediction.sock
diagnostic_socket=$socket_dir/diagnostics.sock
controller_pid=''
model_pid=''
cleanup() {
  trap '' INT TERM
  if [[ -n "$model_pid" ]]; then
    kill -TERM "$model_pid" 2>/dev/null || true
    wait "$model_pid" 2>/dev/null || true
  fi
  if [[ -n "$controller_pid" ]]; then
    kill -TERM "$controller_pid" 2>/dev/null || true
    wait "$controller_pid" 2>/dev/null || true
  fi
  rm -rf -- "$socket_dir"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'Piper SDK 模式：%s，输出目录：%s\n' "$mode" "$output"
( cd "$piper"
  source ./env.sh capture
  sdk_args=()
  if [[ "$mode" == --execute ]]; then sdk_args+=(--execute); fi
  exec python -m data_collect.cli.umi_piper_sdk --setup "$runtime_setup" \
    --socket "$socket_path" --control-log "$output/control_events.jsonl" \
    --diagnostic-socket "$diagnostic_socket" "${sdk_args[@]}"
) > >(tee -a "$output/sdk_controller.log") 2>&1 &
controller_pid=$!
for _ in $(seq 1 600); do
  [[ -S "$socket_path" ]] && break
  if ! kill -0 "$controller_pid" 2>/dev/null; then
    printf 'SDK 控制器启动失败，请查看上面的反馈。\n' >&2
    result=0
    wait "$controller_pid" || result=$?
    controller_pid=''
    if [[ "$result" == 0 ]]; then result=2; fi
    exit "$result"
  fi
  sleep .1
done
if [[ ! -S "$socket_path" ]]; then
  printf 'SDK 控制器启动超时。\n' >&2
  exit 1
fi

cd "$umi"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
# Use the CUDA driver libraries provided by the host system.  The old
# training_49 partial runtime bundles a different NVIDIA driver build and
# causes CUDA error 803 on the current host driver.
unset LD_LIBRARY_PATH
export HF_HUB_OFFLINE=1 WANDB_MODE=disabled OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
model_args=(
  --checkpoint "$checkpoint" --mask "$mask" --setup "$runtime_setup"
  --camera /dev/video0 --can can0 --stream-socket "$socket_path"
  --preview-port "$dashboard_port" --diagnostic-socket "$diagnostic_socket"
  --output "$output" --duration "$duration"
)
if [[ "$disable_inference_augmentations" == 1 ]]; then
  model_args+=(--disable-inference-augmentations)
fi
if [[ "$record_enabled" == 1 ]]; then
  model_args+=(--record-dir "$record_dir")
fi
model_args+=("${match_args[@]}")
"$model_python" scripts/piper_live_observation.py \
  "${model_args[@]}" \
  > >(tee -a "$output/model.log") 2>&1 &
model_pid=$!
wait "$model_pid"
model_pid=''
