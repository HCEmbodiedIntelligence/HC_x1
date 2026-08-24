#!/usr/bin/env bash
# OmniHand O10 RS485 灵巧手桥接（Jetson 板载 ttyTHS + GPIO，与 hc_tj 臂桥接独立）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_WS="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LAUNCH_FILE="${SCRIPT_DIR}/launch/omnihand_o10_rs485_bridge.launch.py"

PREPARE_ONLY="${RS485_HAND_PREPARE_ONLY:-false}"
SKIP_UART_CONFIG="${RS485_HAND_SKIP_UART_CONFIG:-false}"
LAUNCH_ARGS=()

for arg in "$@"; do
  case "${arg}" in
    --prepare-only)
      PREPARE_ONLY=true
      ;;
    --skip-uart-config)
      SKIP_UART_CONFIG=true
      ;;
    *)
      LAUNCH_ARGS+=("${arg}")
      ;;
  esac
done
export RS485_HAND_PREPARE_ONLY="${PREPARE_ONLY}"
export RS485_HAND_SKIP_UART_CONFIG="${SKIP_UART_CONFIG}"
set -- "${LAUNCH_ARGS[@]}"

configure_jetson_uart() {
  if [[ "${SKIP_UART_CONFIG}" == true ]]; then
    return
  fi
  if ! command -v busybox >/dev/null 2>&1; then
    echo "[rs485-hand] 错误: 未找到 busybox，无法配置 Jetson UART 引脚"
    exit 1
  fi

  echo "[rs485-hand] 配置 Jetson UART 引脚复用寄存器..."
  sudo busybox devmem 0x0243d070 w 0x00000400
  sudo busybox devmem 0x0243d078 w 0x00000458
}

can_open_port() {
  local port="$1"
  [[ -e "$port" ]] || return 1
  python3 -c "import os, sys; fd=os.open(sys.argv[1], os.O_RDWR|os.O_NOCTTY); os.close(fd)" "$port" 2>/dev/null
}

in_dialout_group() {
  id -Gn 2>/dev/null | grep -qw dialout && return 0
  getent group dialout | grep -qw "\\b${USER}\\b"
}

ensure_serial_access() {
  local ports=(/dev/ttyTHS0 /dev/ttyTHS2)
  for port in "${ports[@]}"; do
    [[ -e "$port" ]] || continue
    if can_open_port "$port"; then
      continue
    fi
    if [[ -n "${RS485_HAND_IN_DIALOUT:-}" ]]; then
      echo "[rs485-hand] 错误: 仍无法打开 ${port} (Permission denied)"
      echo "  临时修复: sudo chmod 666 ${port}"
      echo "  永久修复: sudo ${TELEOP_WS}/scripts/setup_ttyths_permissions.sh"
      exit 1
    fi
    if in_dialout_group; then
      echo "[rs485-hand] ${port} 需要 dialout 组，当前 shell 未生效，使用 sg dialout 启动..."
      export RS485_HAND_IN_DIALOUT=1
      local -a relaunch=()
      local arg
      for arg in "$@"; do
        [[ -n "$arg" ]] && relaunch+=("$arg")
      done
      if ((${#relaunch[@]})); then
        exec sg dialout -c "RS485_HAND_IN_DIALOUT=1 exec bash $(printf '%q' "$0") $(printf '%q ' "${relaunch[@]}")"
      else
        exec sg dialout -c "RS485_HAND_IN_DIALOUT=1 exec bash $(printf '%q' "$0")"
      fi
    fi
    echo "[rs485-hand] 错误: 无法访问 ${port}"
    echo "  请执行: sudo usermod -aG dialout ${USER} && 重新登录"
    echo "  或临时: sudo chmod 666 ${port}"
    exit 1
  done
}

if [[ ! -f "${TELEOP_WS}/install/setup.bash" ]]; then
  echo "[rs485-hand] 工作空间未编译，请先:"
  echo "  colcon build --packages-select omnihand_o10_rs485_bridge"
  exit 1
fi

configure_jetson_uart
ensure_serial_access "$@"

if [[ "${PREPARE_ONLY}" == true ]]; then
  echo "[rs485-hand] UART 引脚与串口访问检查完成"
  exit 0
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${TELEOP_WS}/install/setup.bash"
set -u

echo "[rs485-hand] 启动: ros2 launch ${LAUNCH_FILE}"
exec ros2 launch "${LAUNCH_FILE}" "$@"
