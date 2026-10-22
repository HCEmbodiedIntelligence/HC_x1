#!/usr/bin/env bash
# hc_tj 上半身真机一键启动：Marvin M6 CCS 双臂 + 腰部（同一个节点）
# 底盘另起 ./run_chassis_bridge.sh
#
# 透传 launch 参数，例如双臂未接线时单独调腰：
#   ./run_hc_tj.sh arms:=false
# 仅启动双臂：
#   ./run_hc_tj.sh waist:=false
# 双臂运动、腰部状态只读：
#   ./run_hc_tj.sh waist:=true waist_control:=false
# 左臂只读、右臂可控：
#   ./run_hc_tj.sh left_arm_control:=false
# 双臂只读（反馈保留）：
#   ./run_hc_tj.sh arms_control:=false

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_WS="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MARVIN_SDK="${MARVIN_SDK_PATH:-${TELEOP_WS}/vendor/marvin}"
LAUNCH_FILE="${SCRIPT_DIR}/launch/hc_tj.launch.py"
WAIST_ENABLED=true
WAIST_CONTROL_ENABLED=true
ARMS_ENABLED=true
ARMS_CONTROL_ENABLED=true
LEFT_ARM_CONTROL_ENABLED=true
RIGHT_ARM_CONTROL_ENABLED=true

for arg in "$@"; do
  case "${arg,,}" in
    arms:=false|arms:=0|arms:=no|arms:=off)
      ARMS_ENABLED=false
      ;;
    arms_control:=false|arms_control:=0|arms_control:=no|arms_control:=off)
      ARMS_CONTROL_ENABLED=false
      ;;
    left_arm_control:=false|left_arm_control:=0|left_arm_control:=no|left_arm_control:=off)
      LEFT_ARM_CONTROL_ENABLED=false
      ;;
    right_arm_control:=false|right_arm_control:=0|right_arm_control:=no|right_arm_control:=off)
      RIGHT_ARM_CONTROL_ENABLED=false
      ;;
    waist:=false|waist:=0|waist:=no|waist:=off)
      WAIST_ENABLED=false
      ;;
    waist_control:=false|waist_control:=0|waist_control:=no|waist_control:=off)
      WAIST_CONTROL_ENABLED=false
      ;;
  esac
done

if [[ ! -f "${MARVIN_SDK}/contrlSDK/libMarvinSDK.so" ]]; then
  echo "[hc_tj] Marvin SDK 未编译，正在构建..."
  (cd "${MARVIN_SDK}/contrlSDK" && make)
fi

if [[ ! -f "${TELEOP_WS}/install/setup.bash" ]]; then
  echo "[HC_X1] 硬件工作空间未编译: ${TELEOP_WS}"
  echo "[hc_tj] 请先执行:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  cd ${TELEOP_WS} && colcon build --packages-select hc_tj_marvin_bridge_cpp"
  exit 1
fi

# 启用腰部时，腰部反馈是 joint_states 的发布前提。
if [[ "${WAIST_ENABLED}" == true ]] &&
   ! redis-cli -h 127.0.0.1 -p 6379 ping >/dev/null 2>&1; then
  echo "[hc_tj] 腰部 Redis (127.0.0.1:6379) 不可达，请先启动腰部控制进程:"
  echo "  systemctl is-active watchdog-waist-control.service"
  echo "[hc_tj] 仅启动双臂可执行: ./run_arms_only.sh"
  exit 1
fi

# ROS setup.bash 会引用可能未设置的 AMENT_* 变量，需临时关闭 nounset
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${TELEOP_WS}/install/setup.bash"
set -u

export MARVIN_SDK_PATH="${MARVIN_SDK}"

if [[ "${ARMS_ENABLED}" == true ]]; then
  if [[ "${ARMS_CONTROL_ENABLED}" == false ]]; then
    echo "[hc_tj] 双臂模式：状态只读（不切换控制模式、不回零、不下发关节命令）"
  else
    echo "[hc_tj] 机械臂控制：左臂=$([[ "${LEFT_ARM_CONTROL_ENABLED}" == true ]] && echo 开启 || echo 只读)，右臂=$([[ "${RIGHT_ARM_CONTROL_ENABLED}" == true ]] && echo 开启 || echo 只读)"
  fi
fi
if [[ "${WAIST_ENABLED}" == true && "${WAIST_CONTROL_ENABLED}" == false ]]; then
  echo "[hc_tj] 上半身模式：Marvin 双臂 + 腰部状态只读（不下发腰部命令）"
elif [[ "${WAIST_ENABLED}" == true ]]; then
  echo "[hc_tj] 上半身模式：双臂 + 腰部"
else
  echo "[hc_tj] 上半身模式：仅 Marvin 双臂（腰部关闭）"
fi
echo "[hc_tj] Marvin SDK: ${MARVIN_SDK}"
echo "[hc_tj] 启动: ros2 launch ${LAUNCH_FILE} $*"
exec ros2 launch "${LAUNCH_FILE}" "$@"
