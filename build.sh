#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ARCH="$(uname -m)"

if [[ "${ARCH}" != "aarch64" && "${HC_X1_ALLOW_CROSS_BUILD:-0}" != "1" ]]; then
  echo "[HC_X1 ERR] 厂商 SDK 是 aarch64 版本，当前架构为 ${ARCH}。" >&2
  echo "请在 X1 Jetson 上构建；仅做交叉构建检查时可设置 HC_X1_ALLOW_CROSS_BUILD=1。" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[HC_X1 ERR] 未找到 ROS 2 Humble: /opt/ros/humble/setup.bash" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u

export MARVIN_SDK_PATH="${MARVIN_SDK_PATH:-${PROJECT_ROOT}/vendor/marvin}"
export OMNIHAND_SDK_ROOT="${OMNIHAND_SDK_ROOT:-${PROJECT_ROOT}/vendor/omnihand}"
export ANGSTRONG_SDK_LIBRARY="${ANGSTRONG_SDK_LIBRARY:-${PROJECT_ROOT}/vendor/angstrong}"
export BERXEL_SDK_LIBRARY="${BERXEL_SDK_LIBRARY:-${PROJECT_ROOT}/vendor/berxel}"

cd "${PROJECT_ROOT}"
# Camera conversion and compression are in the realtime path.  Keep production
# builds optimized unless the caller explicitly supplies another build type.
CMAKE_BUILD_TYPE="${HC_X1_CMAKE_BUILD_TYPE:-Release}"
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
