#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

namespace sw::redis {
class Redis;
}

namespace hc_tj_marvin_bridge {

struct ArmSideConfig {
  char sdk_arm{'A'};
  std::vector<std::string> arm_joints;
  std::array<double, 7> rest_j_pos{};
};

// 腰部走 Redis 对接厂商控制进程的 action 30 (MOVE_SERVOJ)，与 Marvin 双臂 SDK 无关。
struct WaistConfig {
  bool enabled{false};
  std::vector<std::string> joints;
  std::string redis_host{"127.0.0.1"};
  int redis_port{6379};
  std::string redis_key_command{"waist_move"};
  std::string redis_key_status{"waist_status"};
  double command_rate_hz{100.0};
  double max_velocity_deg_s{20.0};
  int servo_timeout_ms{100};
  double cmd_timeout_sec{0.25};
  double state_timeout_sec{0.5};
  bool status_in_degrees{true};
  std::vector<double> limit_lower_deg;
  std::vector<double> limit_upper_deg;
  // 回零目标，配置里写弧度（与双臂 rest_j_pos 及 vr_configs 一致），载入时转度
  std::vector<double> rest_j_pos_deg;
  double home_timeout_sec{20.0};
  double home_tolerance_deg{0.5};
};

class HcTjMarvinBridgeNode : public rclcpp::Node {
 public:
  explicit HcTjMarvinBridgeNode(const std::string& config_path = "");

  ~HcTjMarvinBridgeNode() override;

 private:
  void load_config(const std::string& config_path);
  void build_joint_name_map();
  void setup_io_teleop_interfaces();

  bool connect_robot();
  bool clear_arm_errors();
  bool setup_control_mode();
  bool verify_control_mode();
  bool initialize_kinematics();

  void publish_joint_states();
  void publish_end_effector_feedback(
      const std::array<std::array<double, 7>, 2>& joint_positions_deg,
      const std::array<std::array<double, 6>, 2>& cartesian_wrenches);
  void joint_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void home_robot();
  void publish_hardware_ready(bool ready);
  void reset_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                              std_srvs::srv::Trigger::Response::SharedPtr response);

  void validate_waist_config() const;
  void start_waist_thread();
  void stop_waist_thread();
  void waist_thread_loop();
  void waist_poll_status();
  void waist_send_servoj();
  void latch_waist_target(const sensor_msgs::msg::JointState& msg);
  std::vector<double> clamp_waist_deg(const std::vector<double>& deg);
  void home_waist();
  bool waist_reached(const std::vector<double>& target_deg, double tolerance_deg);

  std::mutex sdk_mutex_;

  std::array<unsigned char, 4> robot_ip_{192, 168, 10, 190};
  std::string hw_control_mode_{"position"};
  int joint_vel_ratio_{100};
  int joint_acc_ratio_{100};
  double control_rate_hz_{100.0};
  int reset_interpolation_steps_{100};
  int reset_interpolation_ms_{5};
  bool auto_home_on_start_{true};
  bool auto_home_done_{false};
  bool connect_arms_{true};
  bool left_arm_control_enabled_{true};
  bool right_arm_control_enabled_{true};
  bool waist_control_enabled_{false};
  std::string kinematics_config_path_;
  bool kinematics_ready_{false};

  std::vector<std::string> all_joint_names_;
  std::unordered_map<std::string, std::pair<std::string, int>> joint_name_map_;

  ArmSideConfig left_cfg_;
  ArmSideConfig right_cfg_;

  std::array<double, 7> cmd_arm_deg_left_{};
  std::array<double, 7> cmd_arm_deg_right_{};

  bool robot_connected_{false};
  bool cmd_target_initialized_{false};
  std::atomic<bool> homing_in_progress_{false};

  // 发布顺序 = 双臂关节 + 腰部关节，与 controller_v2.yml 的 free_joints 一致。
  // arms:=false 的仅腰调试模式下 arm_joint_count_ 为 0，只发腰部。
  WaistConfig waist_cfg_;
  std::unordered_set<std::string> waist_joint_set_;
  std::vector<std::string> published_joint_names_;
  size_t arm_joint_count_{0};

  std::unique_ptr<sw::redis::Redis> waist_redis_;
  std::thread waist_thread_;
  std::atomic<bool> waist_thread_running_{false};
  uint64_t waist_task_id_{0};

  std::mutex waist_target_mutex_;
  std::vector<double> waist_target_deg_;
  bool waist_target_valid_{false};
  bool waist_streaming_{false};
  std::chrono::steady_clock::time_point waist_target_stamp_;

  // 回零期间改用 waist_home_target_deg_，并挡掉 joint_cmd 的锁存，避免两个目标打架
  std::atomic<bool> waist_homing_{false};
  std::vector<double> waist_home_target_deg_;

  std::mutex waist_state_mutex_;
  std::vector<double> waist_state_rad_;
  bool waist_state_received_{false};
  std::chrono::steady_clock::time_point waist_state_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_wrench_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hardware_ready_pub_;
  std::atomic<bool> hardware_ready_{false};
};

}  // namespace hc_tj_marvin_bridge
