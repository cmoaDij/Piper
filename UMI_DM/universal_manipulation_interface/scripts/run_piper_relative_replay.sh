#!/usr/bin/env bash
set -eo pipefail

umi_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
piper_root=$(cd -- "$umi_root/../.." && pwd)/Piper/Data_Collect
data_python=${UMI_DATA_PYTHON:-/home/l1/miniconda3/envs/umi/bin/python}

input_path=""
episode=0
for ((index = 1; index <= $#; index += 1)); do
    case "${!index}" in
        --input)
            next=$((index + 1))
            input_path=${!next}
            ;;
        --input=*)
            input_path=${!index#--input=}
            ;;
        --episode)
            next=$((index + 1))
            episode=${!next}
            ;;
        --episode=*)
            episode=${!index#--episode=}
            ;;
    esac
done

replay_input="$input_path"
temporary_input=""
if [[ -z "$replay_input" || "$replay_input" != *.npz ]]; then
    [[ -x "$data_python" ]] || {
        echo "缺少可读取 Zarr 的 Python：$data_python；可设置 UMI_DATA_PYTHON。" >&2
        exit 2
    }
    temporary_input=$(mktemp --suffix=.npz /tmp/piper_relative_replay_XXXXXX)
    trap 'rm -f -- "$temporary_input"' EXIT
    export_args=(--episode "$episode" --output "$temporary_input")
    [[ -n "$input_path" ]] && export_args+=(--input "$input_path")
    # The exporter uses the standalone UMI environment (Python 3.9).  The
    # caller may already have sourced Piper's capture environment (Python
    # 3.11), whose PYTHONPATH/LD_LIBRARY_PATH would make NumPy load the wrong
    # binary extension.  Keep only the UMI source path for this subprocess.
    env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 \
        PYTHONPATH="$umi_root" "$data_python" \
        "$umi_root/scripts/export_piper_replay_trajectory.py" "${export_args[@]}"
    replay_input="$temporary_input"
fi

source "$piper_root/env.sh" capture
set +e
controller_args=("$@" --input "$replay_input")
[[ -n "$input_path" ]] && controller_args+=(--source-dataset "$input_path")
python "$umi_root/scripts/piper_relative_replay.py" "${controller_args[@]}"
status=$?
set -e
exit "$status"
