#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[HC_X1 CHECK] ROS 2 Humble 未安装" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"

status=0
check_topic() {
  local topic="$1" role="$2"
  local info count_label count
  info="$(timeout 3 ros2 topic info "${topic}" 2>/dev/null || true)"
  if [[ "${role}" == "publisher" ]]; then
    count_label="Publisher count:"
  else
    count_label="Subscription count:"
  fi
  count="$(awk -v label="${count_label}" 'index($0, label) == 1 {print $3}' <<<"${info}")"
  if [[ ! "${count}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[缺失] ${topic} (${role})"
    status=1
    return
  fi
  echo "[正常] ${topic} (${role}=${count})"
}

echo "[HC_X1 CHECK] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
check_topic /observation/joint_states publisher
check_topic /action/joint_states subscriber
check_topic /action/base/command subscriber
check_topic /action/left_hand/joint_states subscriber
check_topic /action/right_hand/joint_states subscriber
exit "${status}"
