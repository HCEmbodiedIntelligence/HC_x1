#include "omnihand_o10_rs485_bridge/hand_config.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr std::array<const char *, 10> kMotorLabels = {
    "thumb_roll",  "thumb_abad",  "thumb_mcp",  "index_abad",  "index_pip",
    "middle_pip",  "ring_abad",   "ring_pip",   "pinky_abad",  "pinky_pip",
};

std::vector<std::string> rightJointNames16() {
  return {"R_thumb_roll_joint",  "R_thumb_abad_joint",  "R_thumb_mcp_joint",
          "R_thumb_pip_joint",   "R_thumb_dip_joint",   "R_index_abad_joint",
          "R_index_mcp_joint",   "R_index_pip_joint",   "R_index_dip_joint",
          "R_middle_mcp_joint",  "R_middle_pip_joint",  "R_middle_dip_joint",
          "R_ring_abad_joint",   "R_ring_pip_joint",    "R_ring_dip_joint",
          "R_pinky_abad_joint",  "R_pinky_pip_joint",   "R_pinky_dip_joint"};
}

std::vector<std::string> leftJointNames16() {
  return {"L_thumb_roll_joint",  "L_thumb_abad_joint",  "L_thumb_mcp_joint",
          "L_thumb_pip_joint",   "L_thumb_dip_joint",   "L_index_abad_joint",
          "L_index_mcp_joint",   "L_index_pip_joint",   "L_index_dip_joint",
          "L_middle_mcp_joint",  "L_middle_pip_joint",  "L_middle_dip_joint",
          "L_ring_abad_joint",   "L_ring_pip_joint",    "L_ring_dip_joint",
          "L_pinky_abad_joint",  "L_pinky_pip_joint",   "L_pinky_dip_joint"};
}

std::vector<double> zeros(size_t n) { return std::vector<double>(n, 0.0); }

void printMotorArray(const char *title, const std::vector<double> &dirs) {
  std::cout << title << '\n';
  if (dirs.size() != 10) {
    std::cout << "  (无效: 期望 10 轴, 实际 " << dirs.size() << ")\n";
    return;
  }
  const auto motor = omnihand_o10_rs485_bridge::fingerDirectionsToMotorPos(dirs);
  for (size_t i = 0; i < 10; ++i) {
    std::cout << "  [" << i << "] " << std::setw(11) << kMotorLabels[i] << "  motor="
              << motor[i] << "  (mapped=" << std::fixed << std::setprecision(1) << dirs[i]
              << ")\n";
  }
}

void printThumbDiff(const std::vector<double> &right_dirs,
                    const std::vector<double> &left_dirs) {
  if (right_dirs.size() != 10 || left_dirs.size() != 10) {
    return;
  }
  const auto r = omnihand_o10_rs485_bridge::fingerDirectionsToMotorPos(right_dirs);
  const auto l = omnihand_o10_rs485_bridge::fingerDirectionsToMotorPos(left_dirs);
  std::cout << "  拇指 motor 差异 (R - L):\n";
  for (size_t i = 0; i < 3; ++i) {
    std::cout << "    " << kMotorLabels[i] << ": " << static_cast<int>(r[i]) << " - "
              << static_cast<int>(l[i]) << " = " << static_cast<int>(r[i]) - static_cast<int>(l[i])
              << '\n';
  }
}

void runScenario(const char *label, const std::vector<std::string> &names,
                 const std::vector<double> &positions, bool is_left) {
  const auto range = is_left ? omnihand_o10_rs485_bridge::leftFingerRange()
                             : omnihand_o10_rs485_bridge::rightFingerRange();
  std::vector<double> dirs;
  omnihand_o10_rs485_bridge::updateFingerDirectionsFromJointState(
      range, names, positions, dirs);
  printMotorArray(label, dirs);
}

}  // namespace

int main() {
  const auto vr_open = omnihand_o10_rs485_bridge::interpolateVrMotorPos(0.0);
  const auto vr_fist = omnihand_o10_rs485_bridge::interpolateVrMotorPos(1.0);
  const auto vr_half = omnihand_o10_rs485_bridge::interpolateVrMotorPos(0.5);
  assert(vr_open.size() == 10 && vr_fist.size() == 10 && vr_half.size() == 10);
  assert(vr_open[2] == 4096.0 && vr_fist[2] == 66.0 && vr_half[2] == 2081.0);
  assert(omnihand_o10_rs485_bridge::normalizeVrTrigger(
             0.5, false, 0.0, 1.0, 0.0) == 0.5);
  assert(omnihand_o10_rs485_bridge::normalizeVrTrigger(
             0.0, true, 0.0, 1.0, 0.02) == 1.0);
  assert(std::abs(omnihand_o10_rs485_bridge::advanceVrClosure(
                      0.0, 1.0, 0.1, 0.5, 0.5) -
                  0.2) < 1e-9);

  const auto r_names = rightJointNames16();
  const auto l_names = leftJointNames16();
  const auto r_range = omnihand_o10_rs485_bridge::rightFingerRange();
  const auto l_range = omnihand_o10_rs485_bridge::leftFingerRange();

  std::cout << "=== O10 手指映射 / 位置指令排查 ===\n\n";

  std::cout << "[A] 初始化张开 openMotorPos\n";
  std::cout << "  右手: ";
  for (size_t i = 0; i < omnihand_o10_rs485_bridge::rightOpenMotorPos().size(); ++i) {
    std::cout << omnihand_o10_rs485_bridge::rightOpenMotorPos()[i]
              << (i + 1 < omnihand_o10_rs485_bridge::rightOpenMotorPos().size() ? ", " : "\n");
  }
  std::cout << "  左手: ";
  for (size_t i = 0; i < omnihand_o10_rs485_bridge::leftOpenMotorPos().size(); ++i) {
    std::cout << omnihand_o10_rs485_bridge::leftOpenMotorPos()[i]
              << (i + 1 < omnihand_o10_rs485_bridge::leftOpenMotorPos().size() ? ", " : "\n");
  }
  std::cout << '\n';

  std::cout << "[B] 无 joint_cmd 时: finger_directions 为空 → 100Hz 控制环不下发\n\n";

  std::cout << "[C] 场景: TeleXperience 16 关节全 0 (未启遥操作常见)\n";
  std::vector<double> r_zero_dirs;
  std::vector<double> l_zero_dirs;
  omnihand_o10_rs485_bridge::updateFingerDirectionsFromJointState(
      r_range, r_names, zeros(r_names.size()), r_zero_dirs);
  omnihand_o10_rs485_bridge::updateFingerDirectionsFromJointState(
      l_range, l_names, zeros(l_names.size()), l_zero_dirs);
  printMotorArray("  右手:", r_zero_dirs);
  printMotorArray("  左手:", l_zero_dirs);
  printThumbDiff(r_zero_dirs, l_zero_dirs);
  std::cout << '\n';

  std::cout << "[D] 场景: 外骨骼「张开」位 (各轴 input_range.first)\n";
  std::vector<double> r_open = zeros(r_names.size());
  std::vector<double> l_open = zeros(l_names.size());
  for (size_t i = 0; i < r_names.size(); ++i) {
    if (r_names[i].find("thumb_roll") != std::string::npos) {
      r_open[i] = r_range[0].input_range.first;
    } else if (r_names[i].find("thumb_abad") != std::string::npos) {
      r_open[i] = r_range[1].input_range.first;
    } else if (r_names[i].find("thumb_mcp") != std::string::npos) {
      r_open[i] = r_range[2].input_range.first;
    }
  }
  for (size_t i = 0; i < l_names.size(); ++i) {
    if (l_names[i].find("thumb_roll") != std::string::npos) {
      l_open[i] = l_range[0].input_range.first;
    } else if (l_names[i].find("thumb_abad") != std::string::npos) {
      l_open[i] = l_range[1].input_range.first;
    } else if (l_names[i].find("thumb_mcp") != std::string::npos) {
      l_open[i] = l_range[2].input_range.first;
    }
  }
  runScenario("  右手 (仅拇指三关节设为 open):", r_names, r_open, false);
  runScenario("  左手 (仅拇指三关节设为 open):", l_names, l_open, true);
  std::cout << '\n';

  std::cout << "[F] 映射表拇指 input/output 范围\n";
  for (size_t i = 0; i < 3; ++i) {
    std::cout << "  R " << kMotorLabels[i] << "  in=[" << r_range[i].input_range.first << ", "
              << r_range[i].input_range.second << "]  out=[" << r_range[i].output_range.first
              << ", " << r_range[i].output_range.second << "]\n";
    std::cout << "  L " << kMotorLabels[i] << "  in=[" << l_range[i].input_range.first << ", "
              << l_range[i].input_range.second << "]  out=[" << l_range[i].output_range.first
              << ", " << l_range[i].output_range.second << "]\n";
  }

  return 0;
}
