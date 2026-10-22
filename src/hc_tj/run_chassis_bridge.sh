#!/usr/bin/env bash
# hc_tj 底盘 Redis 桥接启动

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_WS="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LAUNCH_FILE="${SCRIPT_DIR}/hc_tj_body_bridge/launch/chassis_bridge.launch.py"

if ! python3 -c "import redis" 2>/dev/null; then
  echo "[chassis_bridge] 安装 Python 依赖 redis..."
  python3 -m pip install --user redis
fi

if [[ ! -f "${TELEOP_WS}/install/setup.bash" ]]; then
  echo "[chassis_bridge] 工作空间未编译，正在构建 hc_tj_body_bridge..."
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  cd "${TELEOP_WS}"
  colcon build --packages-select hc_tj_body_bridge
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${TELEOP_WS}/install/setup.bash"
set -u

if ! redis-cli -h 192.168.0.3 ping >/dev/null 2>&1; then
  echo "[chassis_bridge] 警告: 底盘 Redis 192.168.0.3 不可达"
  echo "[chassis_bridge] 请先确认: redis-cli -h 192.168.0.3 ping"
fi

echo "[chassis_bridge] 启动底盘桥接"
echo "[chassis_bridge] 配置: hc_tj_body_bridge/config/body.yaml"
exec ros2 launch "${LAUNCH_FILE}" "$@"
