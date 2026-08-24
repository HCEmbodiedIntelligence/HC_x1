#!/usr/bin/env bash
# hc_tj 相机：启动 camera_driver + 桥接到 io_teleop CompressedImage (MJPEG)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_WS="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ANGSTRONG_SDK="${ANGSTRONG_SDK_LIBRARY:-${TELEOP_WS}/vendor/angstrong}"
BERXEL_SDK="${BERXEL_SDK_LIBRARY:-${TELEOP_WS}/vendor/berxel}"
FASTDDS_PROFILE="${HC_TJ_FASTDDS_PROFILE:-${SCRIPT_DIR}/hc_tj_camera_bridge/config/fastdds_camera_mtu_safe.xml}"

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "[camera_bridge] 缺少文件: $1" >&2
    exit 1
  fi
}

require_file "/opt/ros/humble/setup.bash"
require_file "${ANGSTRONG_SDK}/libs/libAngstrongCameraSdk.so"
require_file "${BERXEL_SDK}/libs/libBerxelHawk.so"
require_file "${FASTDDS_PROFILE}"

if [[ ! -f "${TELEOP_WS}/install/setup.bash" ]]; then
  echo "[camera_bridge] 工作空间未编译，正在构建 hc_tj_camera_bridge..."
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  cd "${TELEOP_WS}"
  colcon build --packages-select hc_tj_camera_bridge
fi

require_file "${TELEOP_WS}/install/setup.bash"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${TELEOP_WS}/install/setup.bash"
set -u

export ANGSTRONG_SDK_LIBRARY="${ANGSTRONG_SDK}"
export BERXEL_SDK_LIBRARY="${BERXEL_SDK}"
export LD_LIBRARY_PATH="${ANGSTRONG_SDK}/libs:${BERXEL_SDK}/libs:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_PROFILE}"

echo "[camera_bridge] HC_X1 工作区: ${TELEOP_WS}"
echo "[camera_bridge] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "[camera_bridge] FastDDS 配置: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "[camera_bridge] camera_driver: /cameras/eye/color, /cameras/eye/depth"
echo "[camera_bridge] io_teleop 输出: /io_teleop/camera_head/color, /io_teleop/camera_head/depth, /io_teleop/camera_head/calibration, /io_teleop/camera_head/extrinsics"
exec ros2 launch hc_tj_camera_bridge camera_bridge.launch.py "$@"
