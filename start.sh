#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "用法: $0 [设备:=false] [设备_control:=false] [--no-*] [launch 参数]"
  echo "控制开关: arms_control、left_arm_control、right_arm_control、waist_control、chassis_control"
  echo "设备开关: arms、waist、chassis、hands、camera、head_camera、d405"
  echo "只负责 X1 硬件和 /action、/observation、/control、/status 标准接口；不启动 HC Teleop 中间件或逆解。"
  exit 0
fi
LOG_DIR="${HC_X1_LOG_DIR:-${PROJECT_ROOT}/runtime/session_$(date +'%Y%m%d_%H%M%S')}"
PIDS=()
_CLEANED=0
exec 9>/tmp/hc_x1_hardware.lock
if ! flock -n 9; then
  echo "[HC_X1 ERR] 已有一个 HC_X1 启动器在运行。" >&2
  exit 2
fi

log() { echo -e "\033[1;34m[HC_X1]\033[0m $*"; }
err() { echo -e "\033[1;31m[HC_X1 ERR]\033[0m $*" >&2; }

cleanup() {
  if [[ "${_CLEANED}" -eq 1 ]]; then return; fi
  _CLEANED=1
  log "正在停止 X1 硬件驱动与标准接口节点..."
  local pid
  for pid in "${PIDS[@]}"; do
    kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do wait "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 0' INT TERM

if [[ ! -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  err "尚未构建，请先执行 ${PROJECT_ROOT}/build.sh"
  exit 2
fi
DRIVER_ARGS=("$@")
if pgrep -f "hc_tj_marvin_bridge_node|run_hc_tj_all.sh" >/dev/null 2>&1; then
  err "检测到旧 X1/HC-TJ 硬件驱动仍在运行；请先正常停止旧启动器，避免重复控制真机。"
  exit 2
fi

mkdir -p "${LOG_DIR}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PYTHONUNBUFFERED=1
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export MARVIN_SDK_PATH="${MARVIN_SDK_PATH:-${PROJECT_ROOT}/vendor/marvin}"
export OMNIHAND_SDK_ROOT="${OMNIHAND_SDK_ROOT:-${PROJECT_ROOT}/vendor/omnihand}"
export ANGSTRONG_SDK_LIBRARY="${ANGSTRONG_SDK_LIBRARY:-${PROJECT_ROOT}/vendor/angstrong}"
export BERXEL_SDK_LIBRARY="${BERXEL_SDK_LIBRARY:-${PROJECT_ROOT}/vendor/berxel}"
export HC_X1_CAMERA_CONFIG_DIR="${HC_X1_CAMERA_CONFIG_DIR:-${PROJECT_ROOT}/config/camera}"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/install/setup.bash"
set -u

if [[ " ${DRIVER_ARGS[*]} " != *" --no-d405 "* ]] && \
   [[ " ${DRIVER_ARGS[*]} " != *" --no-camera "* ]] && \
   ! lsusb 2>/dev/null | grep -qiE "RealSense|Intel Corp"; then
  log "未检测到 Intel RealSense D405，自动添加 --no-d405"
  DRIVER_ARGS+=(--no-d405)
fi
# --no-d405 only disables the external D405 pair.  The head camera must still
# publish both the standard color and depth interfaces.  A diagnostic
# color-only run remains available by explicitly setting
# HC_X1_HEAD_VIDEO_ONLY=1 before starting this script.
if [[ " ${DRIVER_ARGS[*],,} " != *" --no-hands "* ]] &&
   [[ " ${DRIVER_ARGS[*],,} " != *" hands:=false "* ]] &&
   [[ " ${DRIVER_ARGS[*],,} " != *" hands:=0 "* ]] &&
   [[ " ${DRIVER_ARGS[*],,} " != *" hands:=no "* ]] &&
   [[ " ${DRIVER_ARGS[*],,} " != *" hands:=off "* ]]; then
  log "在前台完成 OmniHand UART 权限与 pinmux 检查..."
  bash "${PROJECT_ROOT}/src/omnihand_o10_rs485_bridge/run_omnihand_rs485.sh" \
    --prepare-only
  export RS485_HAND_SKIP_UART_CONFIG=true
fi

log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log "日志目录: ${LOG_DIR}"
# Reset the upper-body log before spawning so a reused HC_X1_LOG_DIR cannot
# satisfy readiness or report an error from an earlier launch.
mkdir -p "${LOG_DIR}/drivers"
: >"${LOG_DIR}/drivers/upper_body.log"
log "启动 X1 私有硬件驱动..."
HC_TJ_LOG_DIR="${LOG_DIR}/drivers" setsid bash \
  "${PROJECT_ROOT}/src/hc_tj/run_hc_tj_all.sh" "${DRIVER_ARGS[@]}" \
  >"${LOG_DIR}/hardware.log" 2>&1 &
PIDS+=("$!")

log "启动 X1 低延迟控制话题适配..."
setsid /usr/bin/python3 -u "${PROJECT_ROOT}/hc_x1/control_adapter.py" \
  >"${LOG_DIR}/control_adapter.log" 2>&1 &
PIDS+=("$!")

if [[ " ${DRIVER_ARGS[*],,} " != *" --no-camera "* ]] &&
   [[ " ${DRIVER_ARGS[*],,} " != *" camera:=false "* ]] &&
   [[ " ${DRIVER_ARGS[*],,} " != *" camera:=0 "* ]] &&
   [[ " ${DRIVER_ARGS[*],,} " != *" camera:=no "* ]] &&
   [[ " ${DRIVER_ARGS[*],,} " != *" camera:=off "* ]]; then
  log "启动 X1 独立压缩相机话题适配..."
  setsid /usr/bin/python3 -u "${PROJECT_ROOT}/hc_x1/camera_adapter.py" \
    >"${LOG_DIR}/camera_adapter.log" 2>&1 &
  PIDS+=("$!")
fi

log "启动 X1 100 Hz 关节命令平滑节点..."
setsid /usr/bin/python3 -u "${PROJECT_ROOT}/hc_x1/joint_command_smoother.py" \
  --config "${PROJECT_ROOT}/config/x1_hardware.yaml" \
  >"${LOG_DIR}/joint_smoother.log" 2>&1 &
PIDS+=("$!")

CHECK_ARGS=(--log "${LOG_DIR}/drivers/upper_body.log"
  --timeout "${HC_X1_STARTUP_TIMEOUT_SEC:-60}")
for pid in "${PIDS[@]}"; do CHECK_ARGS+=(--pid "${pid}"); done
for arg in "${DRIVER_ARGS[@]}"; do CHECK_ARGS+=(--launch-arg="${arg}"); done
log "等待硬件初始化与标准关节接口就绪（最多 ${HC_X1_STARTUP_TIMEOUT_SEC:-60} 秒）..."
if /usr/bin/python3 -u "${PROJECT_ROOT}/hc_x1/startup_check.py" "${CHECK_ARGS[@]}"; then
  :
else
  status=$?
  err "HC_X1 启动失败；请先排除上方故障。正在清理本次启动的进程。"
  exit "${status}"
fi

log "HC_X1 已启动；标准接口为 /action/*、/observation/*、/control/*、/status/*。按 Ctrl+C 只停止 X1 硬件侧。"
set +e
wait -n "${PIDS[@]}"
status=$?
set -e
if [[ ${status} -ne 0 ]]; then err "子进程异常退出，请检查 ${LOG_DIR}"; fi
exit "${status}"
