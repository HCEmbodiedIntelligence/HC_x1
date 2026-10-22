#!/usr/bin/env bash
# hc_tj 真机一键启动：上半身 + 灵巧手 + 底盘 + 原头部相机 + 双 D405
#
# 用法:
#   ./run_hc_tj_all.sh
#   ./run_hc_tj_all.sh --no-hands --no-chassis
#   ./run_hc_tj_all.sh arms:=false          # 透传给 run_hc_tj.sh
#   ./run_hc_tj_all.sh left_arm_control:=false chassis_control:=false
#
# 日志目录: /tmp/hc_tj_logs_<pid>/（可通过 HC_TJ_LOG_DIR 覆盖）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_WS="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HAND_SCRIPT="${TELEOP_WS}/src/omnihand_o10_rs485_bridge/run_omnihand_rs485.sh"
D405_SCRIPT="${TELEOP_WS}/src/hc_d405_stereo_camera/run_camera.sh"

ENABLE_CAMERA=true
ENABLE_D405=true
ENABLE_CHASSIS=true
ENABLE_HANDS=true
ENABLE_WAIST=true
UPPER_BODY_ARGS=()
D405_ARGS=()
CHASSIS_ARGS=()

usage() {
  cat <<'EOF'
hc_tj 真机一键启动

选项:
  --no-camera    不启动任何相机（原头部相机和双 D405）
  --no-head-camera  不启动原 camera_head 相机
  --no-d405      不启动双 D405
  --no-d405-left   左臂 D405 未安装
  --no-d405-right  右臂 D405 未安装
  --no-chassis   不启动底盘 Redis 桥接
  --no-hands     不启动 OmniHand RS485 灵巧手桥接
  chassis:=false       不启动底盘（等价于 --no-chassis）
  chassis_control:=false  底盘只读：保留状态，不下发速度
  hands:=false         不启动灵巧手（等价于 --no-hands）
  camera:=false        不启动任何相机（等价于 --no-camera）
  head_camera:=false   不启动头部相机
  d405:=false          不启动双 D405
  -h, --help     显示帮助

其余参数透传给 run_hc_tj.sh（如 arms:=false、waist_control:=false、
left_arm_control:=false、right_arm_control:=false）。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-camera)
      ENABLE_CAMERA=false
      ENABLE_D405=false
      shift
      ;;
    --no-head-camera)
      ENABLE_CAMERA=false
      shift
      ;;
    --no-d405)
      ENABLE_D405=false
      shift
      ;;
    --no-d405-left)
      D405_ARGS+=(left_enabled:=false)
      shift
      ;;
    --no-d405-right)
      D405_ARGS+=(right_enabled:=false)
      shift
      ;;
    --no-chassis)
      ENABLE_CHASSIS=false
      shift
      ;;
    --no-hands)
      ENABLE_HANDS=false
      shift
      ;;
    camera:=false|camera:=0|camera:=no|camera:=off)
      ENABLE_CAMERA=false
      ENABLE_D405=false
      shift
      ;;
    head_camera:=false|head_camera:=0|head_camera:=no|head_camera:=off)
      ENABLE_CAMERA=false
      shift
      ;;
    d405:=false|d405:=0|d405:=no|d405:=off)
      ENABLE_D405=false
      shift
      ;;
    hands:=false|hands:=0|hands:=no|hands:=off)
      ENABLE_HANDS=false
      shift
      ;;
    chassis:=false|chassis:=0|chassis:=no|chassis:=off)
      ENABLE_CHASSIS=false
      shift
      ;;
    chassis_control:=false|chassis_control:=0|chassis_control:=no|chassis_control:=off)
      CHASSIS_ARGS+=(control:=false)
      shift
      ;;
    chassis_control:=true|chassis_control:=1|chassis_control:=yes|chassis_control:=on)
      CHASSIS_ARGS+=(control:=true)
      shift
      ;;
    chassis:=true|chassis:=1|chassis:=yes|chassis:=on)
      ENABLE_CHASSIS=true
      shift
      ;;
    hands:=true|hands:=1|hands:=yes|hands:=on)
      ENABLE_HANDS=true
      shift
      ;;
    camera:=true|camera:=1|camera:=yes|camera:=on)
      ENABLE_CAMERA=true
      ENABLE_D405=true
      shift
      ;;
    head_camera:=true|head_camera:=1|head_camera:=yes|head_camera:=on)
      ENABLE_CAMERA=true
      shift
      ;;
    d405:=true|d405:=1|d405:=yes|d405:=on)
      ENABLE_D405=true
      shift
      ;;
    waist:=false|waist:=0|waist:=no|waist:=off)
      ENABLE_WAIST=false
      UPPER_BODY_ARGS+=("$1")
      shift
      ;;
    waist:=true|waist:=1|waist:=yes|waist:=on)
      ENABLE_WAIST=true
      UPPER_BODY_ARGS+=("$1")
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      UPPER_BODY_ARGS+=("$1")
      shift
      ;;
  esac
done

LOG_DIR="${HC_TJ_LOG_DIR:-/tmp/hc_tj_logs_$$}"
PIDS=()
STARTED=()

mkdir -p "${LOG_DIR}"

log() {
  echo "[hc_tj_all] $*"
}

cleanup() {
  local pid
  if [[ ${#PIDS[@]} -eq 0 ]]; then
    return 0
  fi
  log "正在停止所有子进程..."
  for pid in "${PIDS[@]}"; do
    kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  done
}

on_signal() {
  cleanup
  exit 0
}

trap on_signal INT TERM
trap cleanup EXIT

start_component() {
  local name="$1"
  local script="$2"
  shift 2

  if [[ ! -x "${script}" ]]; then
    chmod +x "${script}"
  fi

  log "启动 ${name}..."
  setsid bash "${script}" "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  STARTED+=("${name}")
  log "${name} PID=${pid}  日志: ${LOG_DIR}/${name}.log"
}

ensure_workspace() {
  local packages=(hc_tj_marvin_bridge_cpp hc_tj_body_bridge)
  if [[ "${ENABLE_CAMERA}" == true ]]; then
    packages+=(hc_tj_camera_bridge)
  fi
  if [[ "${ENABLE_D405}" == true ]]; then
    packages+=(hc_d405_stereo_camera)
  fi
  if [[ "${ENABLE_HANDS}" == true ]]; then
    packages+=(omnihand_o10_rs485_bridge)
  fi

  if [[ -f "${TELEOP_WS}/install/setup.bash" ]]; then
    return 0
  fi

  log "工作空间未编译，正在构建: ${packages[*]}"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  cd "${TELEOP_WS}"
  colcon build --packages-select "${packages[@]}"
}

preflight() {
  if [[ "${ENABLE_WAIST}" == true ]] &&
    ! redis-cli -h 127.0.0.1 -p 6379 ping >/dev/null 2>&1
  then
    log "错误: 腰部 Redis (127.0.0.1:6379) 不可达"
    log "请先启动: systemctl is-active watchdog-waist-control.service"
    log "不接腰部时把 config/hardware.yaml 的 waist.joints 置空"
    exit 1
  fi

  if [[ "${ENABLE_CHASSIS}" == true ]]; then
    if ! redis-cli -h 192.168.0.3 ping >/dev/null 2>&1; then
      log "警告: 底盘 Redis 192.168.0.3 不可达，底盘桥接可能无响应"
    fi
  fi

  if [[ "${ENABLE_CAMERA}" == true ]]; then
    if systemctl is-active --quiet watchdog-camera-driver.service 2>/dev/null; then
      log "警告: watchdog-camera-driver.service 正在运行，与脚本内 camera_driver 可能冲突"
      log "建议: sudo systemctl stop watchdog-camera-driver.service"
    fi
  fi

  if [[ "${ENABLE_CAMERA}" == true ]] &&
    pgrep -f "ros2 launch hc_tj_camera_bridge camera_bridge.launch.py" >/dev/null 2>&1
  then
    log "检测到旧 camera_bridge 进程，正在自动重启刷新..."
    pkill -f "camera_bridge.launch.py" 2>/dev/null || true
    pkill -f "camera_driver" 2>/dev/null || true
    pkill -f "image_jpeg_compressor" 2>/dev/null || true
    pkill -f "depth_png_compressor" 2>/dev/null || true
    pkill -f "camera_calibration_json_bridge" 2>/dev/null || true
    sleep 1
  fi

  if [[ "${ENABLE_D405}" == true ]]; then
    if [[ ! -x "${D405_SCRIPT}" ]]; then
      log "错误: 找不到 D405 启动脚本 ${D405_SCRIPT}"
      exit 1
    fi
    if pgrep -f "/d405_stereo_camera_node" >/dev/null 2>&1; then
      log "错误: hc_d405_stereo_camera 已在运行，拒绝重复占用 D405"
      log "请先停止旧 D405 进程，或使用 --no-d405"
      exit 1
    fi
  fi

  if [[ "${ENABLE_HANDS}" == true && ! -f "${HAND_SCRIPT}" ]]; then
    log "错误: 找不到灵巧手脚本 ${HAND_SCRIPT}"
    exit 1
  fi
}

ensure_workspace
preflight

log "日志目录: ${LOG_DIR}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}（所有组件统一继承）"

if [[ "${ENABLE_HANDS}" == true ]]; then
  # 在前台完成 sudo pinmux 初始化，避免 hands 进入 setsid 后无法读取 sudo 密码。
  log "初始化灵巧手 UART 引脚..."
  bash "${HAND_SCRIPT}" --prepare-only
fi

start_component upper_body "${SCRIPT_DIR}/run_hc_tj.sh" "${UPPER_BODY_ARGS[@]}"

if [[ "${ENABLE_HANDS}" == true ]]; then
  start_component hands "${HAND_SCRIPT}" --skip-uart-config
fi

if [[ "${ENABLE_CHASSIS}" == true ]]; then
  start_component chassis "${SCRIPT_DIR}/run_chassis_bridge.sh" "${CHASSIS_ARGS[@]}"
fi

if [[ "${ENABLE_CAMERA}" == true ]]; then
  start_component camera "${SCRIPT_DIR}/run_camera_bridge.sh"
fi

if [[ "${ENABLE_D405}" == true ]]; then
  start_component d405_camera "${D405_SCRIPT}" "${D405_ARGS[@]}"
fi

log "已全部启动: ${STARTED[*]}"
log "验证: ros2 topic hz /io_teleop/joint_states"
log "验证: ros2 topic hz /io_teleop/camera_head/color"
log "验证: ros2 topic hz /io_teleop/camera_d405_left/color/compressed"
log "验证: ros2 topic hz /io_teleop/camera_d405_right/color/compressed"
log "按 Ctrl+C 停止全部进程"

while true; do
  for i in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
      log "子进程 ${STARTED[$i]} (PID ${PIDS[$i]}) 已退出，查看: ${LOG_DIR}/${STARTED[$i]}.log"
      wait "${PIDS[$i]}" || true
      exit 1
    fi
  done
  sleep 2
done
