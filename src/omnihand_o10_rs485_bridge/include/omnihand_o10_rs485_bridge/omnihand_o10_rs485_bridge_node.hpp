#pragma once

#include "omnihand_o10_rs485_bridge/hand_config.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "io_msgs2/msg/vr_data.hpp"
#include "omnihand/kinematics/omnihand_2025/omnihand_2025_solver.h"
#include "omnihand/omnihand_2025.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace omnihand_o10_rs485_bridge {

struct HandController {
  std::shared_ptr<agilink::omnihand::OmniHand2025> hand;
  std::vector<FingerMapping> finger_range;
  std::vector<double> finger_directions;
  std::vector<int16_t> actual_motor_positions;
  int init_step{0};
  rclcpp::Time last_init_time;
  bool init_done{false};
  bool enabled{false};
  double vr_closure{0.0};
  double vr_target_closure{0.0};
  bool vr_trigger_pressed{false};
};

class OmniHandO10Rs485BridgeNode : public rclcpp::Node {
 public:
  OmniHandO10Rs485BridgeNode();
  ~OmniHandO10Rs485BridgeNode() override = default;

 private:
  bool initHand(HandController &ctrl, agilink::omnihand::HandType hand_type,
                const std::string &port, const char *name);
  void onControlTimer();
  void onInitTimer();
  void onVibrationTimer();
  void onHandJointStateTimer();
  bool stepInitGrasp(HandController &ctrl, const char *name, bool is_left);
  void sendPositions(HandController &ctrl);
  void collectTactileData(HandController &ctrl, std::vector<double> &data,
                          size_t start_idx);
  void onVrData(const io_msgs2::msg::VrData::SharedPtr msg);
  void updateVrHandTarget(HandController &ctrl, double axis,
                          uint8_t digital_trigger, const char *name);
  void updateVrTriggerControl();

  HandController right_;
  HandController left_;

  std::string left_uart_port_;
  std::string right_uart_port_;
  int uart_baudrate_{460800};
  uint8_t hand_device_id_{1};
  bool enable_left_{true};
  bool enable_right_{true};
  bool enable_vibration_feedback_{true};
  std::string vibration_topic_{"/observation/hands/vibration_feedback"};
  double vibration_rate_hz_{10.0};
  std::string hand_joint_state_topic_{"/observation/hands/joint_states"};
  double hand_joint_state_rate_hz_{20.0};
  std::string hand_trigger_topic_{"/observation/hands/triggers"};
  bool ignore_placeholder_finger_cmd_{true};
  double placeholder_joint_threshold_{0.05};
  bool enable_vr_trigger_control_{false};
  std::string vr_data_topic_{"/control/vr/io_data"};
  double vr_axis_min_{0.0};
  double vr_axis_max_{1.0};
  double vr_trigger_deadzone_{0.02};
  double vr_close_time_sec_{0.6};
  double vr_open_time_sec_{0.6};
  double vr_command_timeout_sec_{0.5};
  bool vr_data_received_{false};
  bool vr_timeout_active_{false};
  std::chrono::steady_clock::time_point last_vr_data_time_;
  std::chrono::steady_clock::time_point last_control_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_sub_;
  rclcpp::Subscription<io_msgs2::msg::VrData>::SharedPtr vr_data_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vibration_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hand_joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr hand_trigger_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr vibration_timer_;
  rclcpp::TimerBase::SharedPtr hand_joint_state_timer_;

  agilink::omnihand::o10::OmniHand2025Solver right_solver_{false};
  agilink::omnihand::o10::OmniHand2025Solver left_solver_{true};
};

}  // namespace omnihand_o10_rs485_bridge
