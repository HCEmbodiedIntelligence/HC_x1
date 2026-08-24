#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${SCRIPT_DIR}" == */install/hc_d405_stereo_camera/lib/hc_d405_stereo_camera ]]; then
  TELEOP_WS="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
  PACKAGE_SOURCE="${TELEOP_WS}/src/hc_d405_stereo_camera"
else
  PACKAGE_SOURCE="$(cd "${SCRIPT_DIR}/.." && pwd)"
  TELEOP_WS="$(cd "${PACKAGE_SOURCE}/../.." && pwd)"
fi

FASTDDS_PROFILE="${HC_D405_FASTDDS_PROFILE:-${PACKAGE_SOURCE}/config/fastdds_camera_mtu_safe.xml}"
LOCK_FILE="${HC_D405_LOCK_FILE:-/tmp/hc_d405_stereo_camera.lock}"
PRESENCE_CONFIG="${HC_D405_CAMERA_PRESENCE_CONFIG:-${PACKAGE_SOURCE}/config/camera_presence.env}"

if [[ -f "${PRESENCE_CONFIG}" ]]; then
  # shellcheck disable=SC1090
  source "${PRESENCE_CONFIG}"
fi
export HC_D405_LEFT_ENABLED="${HC_D405_LEFT_ENABLED:-true}"
export HC_D405_RIGHT_ENABLED="${HC_D405_RIGHT_ENABLED:-true}"

# Keep this descriptor open across `exec ros2 launch`, so two launch commands
# cannot race each other and open the same V4L2 devices simultaneously.
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "[hc_d405] 已有 D405 启动实例持有锁 ${LOCK_FILE}，拒绝重复启动" >&2
  exit 1
fi

# Compatibility with an instance launched before the lock was introduced.
if pgrep -f '[d]405_stereo_camera_node' >/dev/null 2>&1; then
  echo "[hc_d405] d405_stereo_camera_node 已在运行，拒绝重复占用相机" >&2
  echo "[hc_d405] 请直接使用现有话题，或先停止旧实例再启动" >&2
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[hc_d405] 缺少 /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -f "${TELEOP_WS}/install/hc_d405_stereo_camera/lib/hc_d405_stereo_camera/d405_stereo_camera_node" ]]; then
  echo "[hc_d405] 首次构建 hc_d405_stereo_camera..."
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  cd "${TELEOP_WS}"
  colcon build --packages-select hc_d405_stereo_camera
fi
if [[ ! -f "${FASTDDS_PROFILE}" ]]; then
  echo "[hc_d405] 缺少 Fast DDS 配置: ${FASTDDS_PROFILE}" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${TELEOP_WS}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_PROFILE}"

echo "[hc_d405] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "[hc_d405] 硬件配置: left=${HC_D405_LEFT_ENABLED}, right=${HC_D405_RIGHT_ENABLED}"
echo "[hc_d405] VR 左: /io_teleop/camera_d405_left/color/compressed"
echo "[hc_d405] VR 右: /io_teleop/camera_d405_right/color/compressed"
echo "[hc_d405] 不发布 /io_teleop/camera_head/color，可与原头部相机同时运行"

exec ros2 launch hc_d405_stereo_camera d405_stereo_camera.launch.py "$@"
