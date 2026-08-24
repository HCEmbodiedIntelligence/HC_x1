#include "omnihand_o10_rs485_bridge/omnihand_o10_rs485_bridge_node.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;

namespace omnihand_o10_rs485_bridge {

namespace {

using agilink::omnihand::Finger;
using agilink::omnihand::HandType;
using agilink::omnihand::OmniHand2025;

bool isPlaceholderFingerCommand(const sensor_msgs::msg::JointState &msg,
                                double threshold) {
  if (msg.position.empty()) {
    return true;
  }
  for (double pos : msg.position) {
    if (std::abs(pos) > threshold) {
      return false;
    }
  }
  return true;
}

}  // namespace

OmniHandO10Rs485BridgeNode::OmniHandO10Rs485BridgeNode()
    : Node("omnihand_o10_rs485_bridge") {
  right_uart_port_ =
      declare_parameter<std::string>("right_uart_port", "/dev/ttyTHS0");
  left_uart_port_ =
      declare_parameter<std::string>("left_uart_port", "/dev/ttyTHS2");
  uart_baudrate_ = declare_parameter<int>("uart_baudrate", 460800);
  hand_device_id_ = static_cast<uint8_t>(declare_parameter<int>("hand_device_id", 1));
  enable_left_ = declare_parameter<bool>("enable_left", true);
  enable_right_ = declare_parameter<bool>("enable_right", true);
  enable_vibration_feedback_ =
      declare_parameter<bool>("enable_vibration_feedback", true);
  vibration_topic_ = declare_parameter<std::string>(
      "vibration_topic", "/io_teleop/vibration_feedback_exo");
  vibration_rate_hz_ = declare_parameter<double>("vibration_rate_hz", 10.0);
  hand_joint_state_topic_ = declare_parameter<std::string>(
      "hand_joint_state_topic", "/io_teleop/hand_joint_states");
  hand_joint_state_rate_hz_ =
      declare_parameter<double>("hand_joint_state_rate_hz", 20.0);
  hand_trigger_topic_ = declare_parameter<std::string>(
      "hand_trigger_topic", "/io_teleop/hand_trigger");
  ignore_placeholder_finger_cmd_ =
      declare_parameter<bool>("ignore_placeholder_finger_cmd", true);
  placeholder_joint_threshold_ =
      declare_parameter<double>("placeholder_joint_threshold", 0.05);
  enable_vr_trigger_control_ =
      declare_parameter<bool>("enable_vr_trigger_control", false);
  vr_data_topic_ =
      declare_parameter<std::string>("vr_data_topic", "/io_teleop/vr_data");
  vr_axis_min_ = declare_parameter<double>("vr_axis_min", 0.0);
  vr_axis_max_ = declare_parameter<double>("vr_axis_max", 1.0);
  vr_trigger_deadzone_ =
      declare_parameter<double>("vr_trigger_deadzone", 0.02);
  vr_close_time_sec_ = declare_parameter<double>("vr_close_time_sec", 0.6);
  vr_open_time_sec_ = declare_parameter<double>("vr_open_time_sec", 0.6);
  vr_command_timeout_sec_ =
      declare_parameter<double>("vr_command_timeout_sec", 0.5);

  if (vr_axis_max_ <= vr_axis_min_) {
    throw std::invalid_argument("vr_axis_max must be greater than vr_axis_min");
  }
  if (vr_close_time_sec_ <= 0.0 || vr_open_time_sec_ <= 0.0 ||
      vr_command_timeout_sec_ <= 0.0) {
    throw std::invalid_argument(
        "VR trigger close/open/timeout durations must be positive");
  }
  if (hand_joint_state_rate_hz_ <= 0.0) {
    throw std::invalid_argument("hand_joint_state_rate_hz must be positive");
  }

  right_.finger_range = rightFingerRange();
  left_.finger_range = leftFingerRange();

  if (enable_right_) {
    if (!initHand(right_, HandType::RIGHT, right_uart_port_, "右手")) {
      throw std::runtime_error("Failed to init right hand on " + right_uart_port_);
    }
    right_.enabled = true;
    right_.last_init_time = now();
  }

  if (enable_left_) {
    if (!initHand(left_, HandType::LEFT, left_uart_port_, "左手")) {
      throw std::runtime_error("Failed to init left hand on " + left_uart_port_);
    }
    left_.enabled = true;
    left_.last_init_time = now();
  }

  hand_trigger_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      hand_trigger_topic_, rclcpp::QoS(20));
  rclcpp::SensorDataQoS vr_qos;
  vr_qos.keep_last(5);
  vr_data_sub_ = create_subscription<io_msgs2::msg::VrData>(
      vr_data_topic_, vr_qos,
      [this](const io_msgs2::msg::VrData::SharedPtr msg) { onVrData(msg); });

  if (enable_vr_trigger_control_) {
    right_.finger_directions = interpolateVrMotorPos(0.0);
    left_.finger_directions = interpolateVrMotorPos(0.0);
    last_control_time_ = std::chrono::steady_clock::now();
  }

  rclcpp::QoS cmd_qos(10);
  cmd_qos.best_effort();

  if (enable_right_ && !enable_vr_trigger_control_) {
    right_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/io_teleop/joint_cmd_finger_right", cmd_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (msg->name.size() != msg->position.size()) {
            return;
          }
          if (ignore_placeholder_finger_cmd_ &&
              isPlaceholderFingerCommand(*msg, placeholder_joint_threshold_)) {
            return;
          }
          updateFingerDirectionsFromJointState(right_.finger_range, msg->name,
                                               msg->position,
                                               right_.finger_directions);
        });
  }

  if (enable_left_ && !enable_vr_trigger_control_) {
    left_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/io_teleop/joint_cmd_finger_left", cmd_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (msg->name.size() != msg->position.size()) {
            return;
          }
          if (ignore_placeholder_finger_cmd_ &&
              isPlaceholderFingerCommand(*msg, placeholder_joint_threshold_)) {
            return;
          }
          updateFingerDirectionsFromJointState(left_.finger_range, msg->name,
                                               msg->position,
                                               left_.finger_directions);
        });
  }

  control_timer_ = create_wall_timer(10ms, [this]() { onControlTimer(); });
  init_timer_ = create_wall_timer(100ms, [this]() { onInitTimer(); });

  hand_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      hand_joint_state_topic_, rclcpp::QoS(20));
  const auto hand_state_period =
      std::chrono::duration<double>(1.0 / hand_joint_state_rate_hz_);
  hand_joint_state_timer_ = create_wall_timer(
      hand_state_period, [this]() { onHandJointStateTimer(); });

  if (enable_vibration_feedback_) {
    rclcpp::QoS vib_qos(10);
    vib_qos.best_effort();
    vibration_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        vibration_topic_, vib_qos);
    const auto vib_period =
        std::chrono::duration<double>(1.0 / vibration_rate_hz_);
    vibration_timer_ =
        create_wall_timer(vib_period, [this]() { onVibrationTimer(); });
  }

  RCLCPP_INFO(
      get_logger(), "OmniHand O10 RS485 bridge ready (100 Hz ctrl%s%s)",
      enable_vibration_feedback_ ? ", vibration feedback ON" : "",
      enable_vr_trigger_control_ ? ", VR trigger control ON" : "");
  if (enable_vr_trigger_control_) {
    RCLCPP_INFO(get_logger(),
                "VR 扳机: %s，左/右 index (食指扳机) axis 分别控制左/右手，"
                "close=%.2fs open=%.2fs timeout=%.2fs",
                vr_data_topic_.c_str(), vr_close_time_sec_, vr_open_time_sec_,
                vr_command_timeout_sec_);
  }
  RCLCPP_INFO(get_logger(),
              "灵巧手实际关节状态: %s (%.1f Hz, 20 active joints, rad)",
              hand_joint_state_topic_.c_str(), hand_joint_state_rate_hz_);
}

bool OmniHandO10Rs485BridgeNode::initHand(HandController &ctrl,
                                          HandType hand_type,
                                          const std::string &port,
                                          const char *name) {
  RCLCPP_INFO(get_logger(), "%s: 串口 %s @ %d", name, port.c_str(),
              uart_baudrate_);

  ctrl.hand = OmniHand2025::createHandByRs485(hand_type, hand_device_id_, port,
                                              uart_baudrate_);
  if (!ctrl.hand) {
    RCLCPP_ERROR(get_logger(), "%s: 创建 SDK 实例失败", name);
    return false;
  }

  if (!ctrl.hand->Init()) {
    RCLCPP_ERROR(get_logger(), "%s: Init 失败，请检查串口与供电", name);
    return false;
  }

  try {
    const auto vendor = ctrl.hand->GetVendorInfo();
    RCLCPP_INFO(get_logger(), "%s: 握手成功 model=%s SW=%s HW=%s DOF=%u", name,
                vendor.productModel.c_str(),
                vendor.softwareVersion.ToString().c_str(),
                vendor.hardwareVersion.ToString().c_str(),
                static_cast<unsigned>(vendor.dof));
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "%s: 读取设备信息失败: %s", name, e.what());
  }

  return true;
}

void OmniHandO10Rs485BridgeNode::sendPositions(HandController &ctrl) {
  if (!ctrl.enabled || !ctrl.init_done || !ctrl.hand) {
    return;
  }
  if (ctrl.finger_directions.size() != 10) {
    return;
  }
  const auto feedback = ctrl.hand->SetAllJointMotorPosi(
      fingerDirectionsToMotorPos(ctrl.finger_directions));
  if (feedback.size() == 10) {
    ctrl.actual_motor_positions = feedback;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "灵巧手位置下发未返回完整的 10 轴实际反馈");
  }
}

void OmniHandO10Rs485BridgeNode::onControlTimer() {
  if (enable_vr_trigger_control_) {
    updateVrTriggerControl();
  }
  if (enable_right_) {
    sendPositions(right_);
  }
  if (enable_left_) {
    sendPositions(left_);
  }
}

bool OmniHandO10Rs485BridgeNode::stepInitGrasp(HandController &ctrl,
                                               const char *name, bool is_left) {
  if (!ctrl.enabled || ctrl.init_done || !ctrl.hand) {
    return ctrl.init_done;
  }

  const double elapsed = (now() - ctrl.last_init_time).seconds();
  if (enable_vr_trigger_control_) {
    if (ctrl.init_step == 0) {
      RCLCPP_INFO(get_logger(), "%s: 扳机模式初始化到标定张开位", name);
      ctrl.hand->SetAllJointMotorPosi(vrTriggerOpenMotorPos());
      ctrl.finger_directions = interpolateVrMotorPos(0.0);
      ctrl.vr_closure = 0.0;
      ctrl.vr_target_closure = 0.0;
      ctrl.last_init_time = now();
      ctrl.init_step = 1;
      return false;
    }
    if (elapsed >= 0.5) {
      RCLCPP_INFO(get_logger(), "%s: 扳机模式初始化完成", name);
      ctrl.init_done = true;
    }
    return ctrl.init_done;
  }

  switch (ctrl.init_step) {
    case 0:
      RCLCPP_INFO(get_logger(), "%s: 执行初始化抓握...", name);
      ctrl.hand->SetAllJointMotorPosi(openMotorPos(is_left));
      ctrl.last_init_time = now();
      ctrl.init_step = 1;
      break;
    case 1:
      if (elapsed >= 1.0) {
        ctrl.hand->SetAllJointMotorPosi(graspMotorPos());
        ctrl.last_init_time = now();
        ctrl.init_step = 2;
      }
      break;
    case 2:
      if (elapsed >= 1.0) {
        ctrl.hand->SetAllJointMotorPosi(openMotorPos(is_left));
        ctrl.last_init_time = now();
        ctrl.init_step = 3;
      }
      break;
    case 3:
      if (elapsed >= 0.5) {
        RCLCPP_INFO(get_logger(), "%s: 初始化完成", name);
        ctrl.init_done = true;
      }
      break;
    default:
      ctrl.init_done = true;
      break;
  }
  return ctrl.init_done;
}

void OmniHandO10Rs485BridgeNode::updateVrHandTarget(
    HandController &ctrl, double axis, uint8_t digital_trigger,
    const char *name) {
  const double target = normalizeVrTrigger(
      axis, digital_trigger != 0, vr_axis_min_, vr_axis_max_,
      vr_trigger_deadzone_);
  const bool pressed = digital_trigger != 0 || target > 0.0;
  if (pressed != ctrl.vr_trigger_pressed) {
    RCLCPP_INFO(get_logger(), "%s扳机%s axis=%.3f target=%.3f", name,
                pressed ? "按下" : "松开", axis, target);
    ctrl.vr_trigger_pressed = pressed;
  }
  ctrl.vr_target_closure = target;
}

void OmniHandO10Rs485BridgeNode::onVrData(
    const io_msgs2::msg::VrData::SharedPtr msg) {
  if (enable_vr_trigger_control_) {
    last_vr_data_time_ = std::chrono::steady_clock::now();
    vr_data_received_ = true;
    vr_timeout_active_ = false;
    if (enable_left_) {
      updateVrHandTarget(left_, msg->l_index_axis, msg->l_index_trigger,
                         "左手食指");
    }
    if (enable_right_) {
      updateVrHandTarget(right_, msg->r_index_axis, msg->r_index_trigger,
                         "右手食指");
    }
  }

  geometry_msgs::msg::PoseArray trigger_msg;
  trigger_msg.header = msg->header;
  if (trigger_msg.header.stamp.sec == 0 &&
      trigger_msg.header.stamp.nanosec == 0) {
    trigger_msg.header.stamp = now();
  }
  trigger_msg.header.frame_id = "vr_hand_triggers_left_right";
  trigger_msg.poses.resize(2);
  // 固定顺序：poses[0]=左手，poses[1]=右手。
  // position.x=模拟量，position.y=数字状态；单位四元数表示无旋转。
  trigger_msg.poses[0].position.x = msg->l_index_axis;
  trigger_msg.poses[0].position.y = static_cast<double>(msg->l_index_trigger);
  trigger_msg.poses[0].orientation.w = 1.0;
  trigger_msg.poses[1].position.x = msg->r_index_axis;
  trigger_msg.poses[1].position.y = static_cast<double>(msg->r_index_trigger);
  trigger_msg.poses[1].orientation.w = 1.0;
  hand_trigger_pub_->publish(trigger_msg);
}

void OmniHandO10Rs485BridgeNode::onHandJointStateTimer() {
  if (right_.actual_motor_positions.empty() &&
      left_.actual_motor_positions.empty()) {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();

  auto append_hand = [this, &msg](
                         const HandController &ctrl,
                         agilink::omnihand::o10::OmniHand2025Solver &solver,
                         const std::vector<FingerMapping> &mapping) {
    std::vector<std::string> names;
    names.reserve(mapping.size());
    for (const auto &item : mapping) {
      names.push_back(item.joint_names.front());
    }
    msg.name.insert(msg.name.end(), names.begin(), names.end());
    if (!ctrl.enabled || ctrl.actual_motor_positions.size() != 10) {
      msg.position.insert(msg.position.end(), 10,
                          std::numeric_limits<double>::quiet_NaN());
      return;
    }
    const std::vector<int> motor_positions(ctrl.actual_motor_positions.begin(),
                                           ctrl.actual_motor_positions.end());
    const auto joint_positions =
        solver.ActuatorInput2ActiveJointPos(motor_positions);
    if (joint_positions.size() == 10) {
      msg.position.insert(msg.position.end(), joint_positions.begin(),
                          joint_positions.end());
    } else {
      msg.position.insert(msg.position.end(), 10,
                          std::numeric_limits<double>::quiet_NaN());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "灵巧手实际电机位置转换关节角失败");
    }
  };

  // 与 TeleXperience 现有双臂/触觉顺序一致：右手 10 轴在前，左手 10 轴在后。
  append_hand(right_, right_solver_, right_.finger_range);
  append_hand(left_, left_solver_, left_.finger_range);
  hand_joint_state_pub_->publish(msg);
}

void OmniHandO10Rs485BridgeNode::updateVrTriggerControl() {
  const auto now_steady = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now_steady - last_control_time_).count();
  last_control_time_ = now_steady;
  dt = std::clamp(dt, 0.0, 0.1);

  if (vr_data_received_) {
    const double age =
        std::chrono::duration<double>(now_steady - last_vr_data_time_).count();
    if (age > vr_command_timeout_sec_) {
      left_.vr_target_closure = 0.0;
      right_.vr_target_closure = 0.0;
      if (!vr_timeout_active_) {
        RCLCPP_WARN(get_logger(),
                    "VR 数据超时 %.3fs，灵巧手自动线性回到张开位", age);
        vr_timeout_active_ = true;
      }
    }
  }

  auto update = [this, dt](HandController &ctrl) {
    if (!ctrl.enabled) {
      return;
    }
    ctrl.vr_closure = advanceVrClosure(
        ctrl.vr_closure, ctrl.vr_target_closure, dt, vr_close_time_sec_,
        vr_open_time_sec_);
    ctrl.finger_directions = interpolateVrMotorPos(ctrl.vr_closure);
  };
  update(left_);
  update(right_);
}

void OmniHandO10Rs485BridgeNode::onInitTimer() {
  const bool right_done = !enable_right_ || stepInitGrasp(right_, "右手", false);
  const bool left_done = !enable_left_ || stepInitGrasp(left_, "左手", true);
  if (right_done && left_done) {
    init_timer_->cancel();
  }
}

void OmniHandO10Rs485BridgeNode::collectTactileData(HandController &ctrl,
                                                    std::vector<double> &data,
                                                    size_t start_idx) {
  static const std::vector<Finger> fingers = {
      Finger::THUMB, Finger::INDEX, Finger::MIDDLE, Finger::RING, Finger::LITTLE};

  try {
    for (size_t i = 0; i < fingers.size(); ++i) {
      const auto sensor_data = ctrl.hand->GetTactileSensorData(fingers[i]);
      int sum = 0;
      for (uint8_t val : sensor_data) {
        sum += static_cast<int>(val);
      }
      double mapped_value = (sum / 2000.0) * 10.0;
      mapped_value = std::clamp(mapped_value, 0.0, 10.0);
      data[start_idx + i] = mapped_value;
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "触觉传感器读取失败: %s", e.what());
  }
}

void OmniHandO10Rs485BridgeNode::onVibrationTimer() {
  if ((!enable_right_ || !right_.init_done) &&
      (!enable_left_ || !left_.init_done)) {
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(10, 0.0);

  if (enable_right_ && right_.init_done) {
    collectTactileData(right_, msg.data, 0);
  }
  if (enable_left_ && left_.init_done) {
    collectTactileData(left_, msg.data, 5);
  }

  vibration_pub_->publish(msg);
}

}  // namespace omnihand_o10_rs485_bridge
