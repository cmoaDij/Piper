#!/usr/bin/env bash
set -euo pipefail

# Build EAIK for the Piper capture runtime.  The upstream PyPI wheel is not
# published for every Python 3.11 platform, while its source build only needs
# Eigen (already provided by Ubuntu's libeigen3-dev package) and pybind11.
data_collect_root="${PIPER_DATA_COLLECT_ROOT:-/home/l1/Piper_All/Piper/Data_Collect}"
python_bin="${data_collect_root}/.runtime/capture/bin/python"
if [[ ! -x "${python_bin}" ]]; then
    echo "capture Python not found: ${python_bin}" >&2
    exit 1
fi

"${python_bin}" -m pip install --upgrade --no-cache-dir urchin pybind11 wheel
build_dir="$(mktemp -d "${TMPDIR:-/tmp}/eaik-build.XXXXXX")"
trap 'rm -rf "${build_dir}"' EXIT
"${python_bin}" -m pip download --no-binary=:all: --no-deps --dest "${build_dir}" EAIK==1.2.2
source_archive="$(find "${build_dir}" -maxdepth 1 -name 'eaik-*.tar.gz' -print -quit)"
if [[ -z "${source_archive}" ]]; then
    echo "EAIK source archive was not downloaded" >&2
    exit 1
fi
mkdir "${build_dir}/wheel"
CXXFLAGS="${CXXFLAGS:-} -I/usr/include/eigen3" \
    "${python_bin}" -m pip wheel --no-deps --no-build-isolation \
    --wheel-dir "${build_dir}/wheel" "${source_archive}"
wheel="$(find "${build_dir}/wheel" -maxdepth 1 -name 'eaik-*.whl' -print -quit)"
"${python_bin}" -m pip install --no-deps "${wheel}"
"${python_bin}" - <<'PY'
import eaik
print(f'EAIK ready: {eaik.__file__}')
PY
