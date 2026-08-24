#!/usr/bin/env bash
# 手指位置指令 / 映射排查
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

echo "=== [1] 离线映射测试 (无需硬件) ==="
ros2 run omnihand_o10_rs485_bridge test_finger_mapping

echo ""
echo "=== [2] 在线 joint_cmd 监听 (需 TeleXperience 或 --publish-test) ==="
echo "  另开终端运行:"
echo "    export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-14}"
echo "    python3 ${WS}/src/omnihand_o10_rs485_bridge/scripts/test_finger_cmd_debug.py"
echo ""
echo "  或发布测试指令 (全零 / open 位):"
echo "    python3 .../test_finger_cmd_debug.py --publish-test zero"
echo "    python3 .../test_finger_cmd_debug.py --publish-test open"
