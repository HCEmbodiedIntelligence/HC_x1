#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace omnihand_o10_rs485_bridge {

struct FingerMapping {
  std::vector<std::string> joint_names;
  std::pair<double, double> input_range;
  std::pair<double, double> output_range;
};

/// 右手 init 张开位姿；左手前两个轴（thumb_roll / thumb_abad）镜像取反
inline const std::vector<int16_t> &rightOpenMotorPos() {
  static const std::vector<int16_t> kPos = {0, 0, 4096, 2000, 4096,
                                            4096, 2000, 4096, 2000, 4096};
  return kPos;
}

inline const std::vector<int16_t> &leftOpenMotorPos() {
  static const std::vector<int16_t> kPos = {4096, 4096, 4096, 2000, 4096,
                                            4096, 2000, 4096, 2000, 4096};
  return kPos;
}

inline const std::vector<int16_t> &openMotorPos(bool is_left) {
  return is_left ? leftOpenMotorPos() : rightOpenMotorPos();
}

inline const std::vector<int16_t> &graspMotorPos() {
  static const std::vector<int16_t> kPos(10, 1000);
  return kPos;
}

/// VR 中指（握把）扳机控制使用的两端位置。两只手均使用实机标定后的同一组电机值。
inline const std::vector<int16_t> &vrTriggerOpenMotorPos() {
  static const std::vector<int16_t> kPos = {
      3800, 4096, 4096, 1899, 4096, 4096, 4096, 4096, 0, 4096};
  return kPos;
}

inline const std::vector<int16_t> &vrTriggerFistMotorPos() {
  static const std::vector<int16_t> kPos = {
      3800, 4096, 66, 1899, 2300, 2300, 1932, 604, 2094, 408};
  return kPos;
}

inline double normalizeVrTrigger(double axis, bool digital_pressed,
                                 double axis_min, double axis_max,
                                 double deadzone) {
  double value = 0.0;
  const double span = axis_max - axis_min;
  if (std::isfinite(axis) && std::abs(span) > 1e-9) {
    value = std::clamp((axis - axis_min) / span, 0.0, 1.0);
  }
  deadzone = std::clamp(deadzone, 0.0, 0.99);
  if (value <= deadzone) {
    // 某些 VR 驱动只更新 uint8 trigger，axis 始终为 0。
    return digital_pressed ? 1.0 : 0.0;
  }
  return std::clamp((value - deadzone) / (1.0 - deadzone), 0.0, 1.0);
}

inline double advanceVrClosure(double current, double target, double dt,
                               double close_time, double open_time) {
  current = std::clamp(current, 0.0, 1.0);
  target = std::clamp(target, 0.0, 1.0);
  if (!std::isfinite(dt) || dt <= 0.0) {
    return current;
  }
  const double duration = target >= current ? close_time : open_time;
  if (!std::isfinite(duration) || duration <= 0.0) {
    return target;
  }
  const double step = dt / duration;
  if (target > current) {
    return std::min(target, current + step);
  }
  return std::max(target, current - step);
}

inline std::vector<double> interpolateVrMotorPos(double closure) {
  closure = std::clamp(closure, 0.0, 1.0);
  const auto &open = vrTriggerOpenMotorPos();
  const auto &fist = vrTriggerFistMotorPos();
  std::vector<double> positions;
  positions.reserve(open.size());
  for (size_t i = 0; i < open.size(); ++i) {
    positions.push_back(static_cast<double>(open[i]) +
                        closure * static_cast<double>(fist[i] - open[i]));
  }
  return positions;
}

inline std::vector<FingerMapping> rightFingerRange() {
  return {
      {{"R_thumb_roll_joint"}, {0.4, 1.121}, {4096, 800}},
      {{"R_thumb_abad_joint"}, {-1.642, 0.045}, {4096, 0}},
      {{"R_thumb_mcp_joint"}, {0.0, 1.0}, {4096, 0}},
      {{"R_index_abad_joint"}, {-0.13, -0.05}, {4096, 0}},
      {{"R_index_pip_joint"}, {0.0, 2.5}, {4096, 0}},
      {{"R_middle_pip_joint"}, {0.0, 2.5}, {4096, 0}},
      {{"R_ring_abad_joint"}, {0.05, 0.13}, {0, 4096}},
      {{"R_ring_pip_joint"}, {0.0, 2.5}, {4096, 0}},
      {{"R_pinky_abad_joint"}, {0.05, 0.13}, {4096, 0}},
      {{"R_pinky_pip_joint"}, {0.0, 2.5}, {4096, 0}},
  };
}

inline std::vector<FingerMapping> leftFingerRange() {
  return {
      {{"L_thumb_roll_joint"}, {-0.4, -1.121}, {4096, 800}},
      {{"L_thumb_abad_joint"}, {-0.045, 1.642}, {4096, 0}},
      {{"L_thumb_mcp_joint"}, {0.0, -1.0}, {4096, 0}},
      {{"L_index_abad_joint"}, {0.05, 0.13}, {4096, 0}},
      {{"L_index_pip_joint"}, {0.0, 2.5}, {4096, 0}},
      {{"L_middle_pip_joint"}, {0.0, 2.5}, {4096, 0}},
      {{"L_ring_abad_joint"}, {-0.13, -0.05}, {0, 4096}},
      {{"L_ring_pip_joint"}, {0.0, 2.5}, {4096, 0}},
      {{"L_pinky_abad_joint"}, {-0.13, 0.05}, {4096, 0}},
      {{"L_pinky_pip_joint"}, {0.0, 2.5}, {4096, 0}},
  };
}

inline void updateFingerDirectionsFromJointState(
    const std::vector<FingerMapping> &finger_range,
    const std::vector<std::string> &msg_names,
    const std::vector<double> &msg_positions,
    std::vector<double> &finger_directions) {
  std::unordered_map<std::string, double> joint_dic;
  for (size_t i = 0; i < msg_names.size(); ++i) {
    if (i < msg_positions.size()) {
      joint_dic[msg_names[i]] = msg_positions[i];
    }
  }

  finger_directions.clear();
  finger_directions.reserve(finger_range.size());
  for (const auto &mapping : finger_range) {
    double sum = 0.0;
    for (const auto &name : mapping.joint_names) {
      const auto it = joint_dic.find(name);
      if (it != joint_dic.end()) {
        sum += it->second;
      }
    }
    const double in_span = mapping.input_range.second - mapping.input_range.first;
    double t = 0.0;
    if (std::abs(in_span) > 1e-9) {
      t = (sum - mapping.input_range.first) / in_span;
      t = std::clamp(t, 0.0, 1.0);
    }
    finger_directions.push_back(mapping.output_range.first +
                                t * (mapping.output_range.second - mapping.output_range.first));
  }
}

inline std::vector<int16_t> fingerDirectionsToMotorPos(
    const std::vector<double> &finger_directions) {
  std::vector<int16_t> positions;
  positions.reserve(finger_directions.size());
  for (const auto pos : finger_directions) {
    positions.push_back(static_cast<int16_t>(pos));
  }
  return positions;
}

}  // namespace omnihand_o10_rs485_bridge
