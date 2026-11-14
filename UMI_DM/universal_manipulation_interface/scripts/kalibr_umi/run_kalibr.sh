#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 1 ]; then
  printf 'Usage: %s /absolute/path/to/kalibr_comparison\n' "$0" >&2
  exit 2
fi
data_dir=$(cd "$1" && pwd)
adapter_dir=$(cd "$(dirname "$0")" && pwd)
for name in gopro_fullres.bag camchain.yaml imu_provisional.yaml target_geometry.yaml charuco_observations.json; do
  if [ ! -f "$data_dir/$name" ]; then printf 'Missing: %s\n' "$data_dir/$name" >&2; exit 1; fi
done
docker run --rm --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e MPLCONFIGDIR=/tmp -e MPLBACKEND=Agg \
  -e UMI_CHARUCO_OBSERVATIONS=/data/charuco_observations.json \
  -v "$data_dir:/data" -v "$adapter_dir:/adapter:ro" \
  --entrypoint /bin/bash umi-kalibr:local -c '
    set -e
    source /catkin_ws/devel/setup.bash
    cd /data
    exec python3 /adapter/charuco_adapter.py \
      --bag /data/gopro_fullres.bag --cams /data/camchain.yaml \
      --imu /data/imu_provisional.yaml --target /data/target_geometry.yaml \
      --dont-show-report
  '
