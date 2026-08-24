#!/usr/bin/env bash
# hc_tj 双臂 + VR 扳机灵巧手模式。
# 腰部只用于补齐 17 维 joint_states：读取反馈，但不下发控制。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_WS="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HAND_SCRIPT="${TELEOP_WS}/src/omnihand_o10_rs485_bridge/run_omnihand_rs485.sh"
LOG_DIR="${HC_TJ_LOG_DIR:-/tmp/hc_tj_arms_hands_logs_$$}"
PIDS=()
NAMES=()

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
mkdir -p "${LOG_DIR}"

log() {
  echo "[arms+hands] $*"
}

cleanup() {
  local pid
  if [[ ${#PIDS[@]} -eq 0 ]]; then
    return
  fi
  log "正在停止双臂与灵巧手..."
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
  shift
  setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  NAMES+=("${name}")
  log "${name} PID=${pid} 日志=${LOG_DIR}/${name}.log"
}

if [[ ! -x "${HAND_SCRIPT}" ]]; then
  log "错误: 找不到灵巧手启动脚本 ${HAND_SCRIPT}"
  exit 1
fi
if [[ ! -f "${TELEOP_WS}/install/setup.bash" ]]; then
  log "错误: 工作空间尚未编译，请先执行 colcon build"
  exit 1
fi
if pgrep -f 'omnihand_o10_rs485_bridge_node' >/dev/null 2>&1; then
  log "错误: 灵巧手桥已经运行，拒绝重复占用串口"
  exit 1
fi

log "模式：Marvin 双臂 + VR 中指（握把）扳机灵巧手（腰部状态只读）"
log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log "左中指扳机控制左手，右中指扳机控制右手；松开线性张开"
log "日志目录: ${LOG_DIR}"

# 前台完成 UART pinmux 和权限检查，再把手桥放入受控子进程。
bash "${HAND_SCRIPT}" --prepare-only

start_component upper_body bash "${SCRIPT_DIR}/run_hc_tj.sh" \
  "$@" arms:=true waist:=true waist_control:=false
start_component hands bash "${HAND_SCRIPT}" --skip-uart-config \
  enable_vr_trigger_control:=true

log "已启动。按 Ctrl+C 同时停止双臂与灵巧手"
while true; do
  for index in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$index]}" 2>/dev/null; then
      log "${NAMES[$index]} 已退出，请查看 ${LOG_DIR}/${NAMES[$index]}.log"
      wait "${PIDS[$index]}" || true
      exit 1
    fi
  done
  sleep 1
done
