#include "hc_tj_marvin_bridge_cpp/hc_tj_marvin_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <thread>

#include <nlohmann/json.hpp>
#include <sw/redis++/redis++.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include "FxRobot.h"
#include "MarvinSDK.h"

namespace hc_tj_marvin_bridge {

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;

// 与 SDK redis_control_robot.cpp 中 action 30 的参数校验保持一致
constexpr int kServoJAction = 30;
constexpr int kServoJArmId = 3;
constexpr double kServoJMaxVelocityUpper = 90.0;
constexpr int kServoJTimeoutMsLower = 10;
constexpr int kServoJTimeoutMsUpper = 1000;
constexpr int kArmModeVerifyAttempts = 20;
constexpr int kArmModeVerifyIntervalMs = 100;
constexpr double kMillimetersToMeters = 0.001;

int sdk_arm_index(char arm) {
  return std::toupper(static_cast<unsigned char>(arm)) == 'B' ? 1 : 0;
}

geometry_msgs::msg::Pose matrix_to_pose(const Matrix4& matrix) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix[0][3] * kMillimetersToMeters;
  pose.position.y = matrix[1][3] * kMillimetersToMeters;
  pose.position.z = matrix[2][3] * kMillimetersToMeters;

  const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
  if (trace > 0.0) {
    const double s = 2.0 * std::sqrt(trace + 1.0);
    pose.orientation.w = 0.25 * s;
    pose.orientation.x = (matrix[2][1] - matrix[1][2]) / s;
    pose.orientation.y = (matrix[0][2] - matrix[2][0]) / s;
    pose.orientation.z = (matrix[1][0] - matrix[0][1]) / s;
  } else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
    const double s =
        2.0 * std::sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]);
    pose.orientation.w = (matrix[2][1] - matrix[1][2]) / s;
    pose.orientation.x = 0.25 * s;
    pose.orientation.y = (matrix[0][1] + matrix[1][0]) / s;
    pose.orientation.z = (matrix[0][2] + matrix[2][0]) / s;
  } else if (matrix[1][1] > matrix[2][2]) {
    const double s =
        2.0 * std::sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]);
    pose.orientation.w = (matrix[0][2] - matrix[2][0]) / s;
    pose.orientation.x = (matrix[0][1] + matrix[1][0]) / s;
    pose.orientation.y = 0.25 * s;
    pose.orientation.z = (matrix[1][2] + matrix[2][1]) / s;
  } else {
    const double s =
        2.0 * std::sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]);
    pose.orientation.w = (matrix[1][0] - matrix[0][1]) / s;
    pose.orientation.x = (matrix[0][2] + matrix[2][0]) / s;
    pose.orientation.y = (matrix[1][2] + matrix[2][1]) / s;
    pose.orientation.z = 0.25 * s;
  }
  return pose;
}

std::vector<double> parse_double_list(const YAML::Node& node) {
  std::vector<double> out;
  if (!node || !node.IsSequence()) {
    return out;
  }
  for (const auto& item : node) {
    out.push_back(item.as<double>());
  }
  return out;
}

// waist_status 的形状随控制进程版本变化过，这里兼容裸数组、{joint|joints|position|q}
// 以及嵌一层 params 的写法。
bool extract_status_joints(const nlohmann::json& data, size_t count,
                           std::vector<double>& out) {
  if (data.is_array() && data.size() >= count) {
    out.clear();
    for (size_t i = 0; i < count; ++i) {
      if (!data[i].is_number()) {
        return false;
      }
      out.push_back(data[i].get<double>());
    }
    return true;
  }
  if (!data.is_object()) {
    return false;
  }
  for (const char* key : {"joint", "joints", "position", "q"}) {
    const auto it = data.find(key);
    if (it != data.end() && it->is_array() && it->size() >= count) {
      return extract_status_joints(*it, count, out);
    }
  }
  const auto params = data.find("params");
  if (params != data.end()) {
    return extract_status_joints(*params, count, out);
  }
  return false;
}

std::array<double, 7> parse_rest_j_pos7(const YAML::Node& node) {
  std::array<double, 7> out{};
  if (!node || !node.IsSequence()) {
    return out;
  }
  for (size_t i = 0; i < out.size() && i < node.size(); ++i) {
    out[i] = node[i].as<double>();
  }
  return out;
}

std::vector<std::string> parse_string_list(const YAML::Node& node) {
  std::vector<std::string> out;
  if (!node || !node.IsSequence()) {
    return out;
  }
  for (const auto& item : node) {
    out.push_back(item.as<std::string>());
  }
  return out;
}

ArmSideConfig parse_arm_side(const YAML::Node& node) {
  ArmSideConfig cfg;
  if (!node) {
    return cfg;
  }
  if (node["sdk_arm"]) {
    const std::string arm = node["sdk_arm"].as<std::string>();
    cfg.sdk_arm = static_cast<char>(arm.empty() ? 'A' : arm[0]);
  }
  cfg.arm_joints = parse_string_list(node["arm_joints"]);
  cfg.rest_j_pos = parse_rest_j_pos7(node["rest_j_pos"]);
  return cfg;
}

void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace

HcTjMarvinBridgeNode::HcTjMarvinBridgeNode(const std::string& config_path)
    : Node("hc_tj_marvin_bridge") {
  std::string resolved_config = config_path;
  if (resolved_config.empty()) {
    this->declare_parameter<std::string>("hardware_config_path", "");
    resolved_config = this->get_parameter("hardware_config_path").as_string();
  }
  if (resolved_config.empty()) {
    const auto share_dir =
        ament_index_cpp::get_package_share_directory("hc_tj_marvin_bridge_cpp");
    resolved_config = share_dir + "/config/hardware.yaml";
  }

  connect_arms_ = this->declare_parameter<bool>("connect_arms", true);
  const bool enable_arms_control =
      this->declare_parameter<bool>("enable_arms_control", true);
  const bool enable_left_arm_control =
      this->declare_parameter<bool>("enable_left_arm_control", true);
  const bool enable_right_arm_control =
      this->declare_parameter<bool>("enable_right_arm_control", true);
  const bool enable_waist = this->declare_parameter<bool>("enable_waist", true);
  const bool enable_waist_control =
      this->declare_parameter<bool>("enable_waist_control", true);

  load_config(resolved_config);
  left_arm_control_enabled_ =
      connect_arms_ && enable_arms_control && enable_left_arm_control;
  right_arm_control_enabled_ =
      connect_arms_ && enable_arms_control && enable_right_arm_control;
  waist_cfg_.enabled = waist_cfg_.enabled && enable_waist;
  waist_control_enabled_ = waist_cfg_.enabled && enable_waist_control;

  // 必须在 launch 参数覆盖腰部开关之后构造发布列表。否则 waist:=false 时
  // published_joint_names_ 仍会残留 3 个腰部关节，publish_joint_states()
  // 会从空的 waist_values 越界读取。
  arm_joint_count_ = connect_arms_ ? all_joint_names_.size() : 0;
  published_joint_names_.clear();
  if (connect_arms_) {
    published_joint_names_ = all_joint_names_;
  }
  if (waist_cfg_.enabled) {
    published_joint_names_.insert(published_joint_names_.end(), waist_cfg_.joints.begin(),
                                  waist_cfg_.joints.end());
  }

  validate_waist_config();
  build_joint_name_map();

  if (!connect_arms_) {
    RCLCPP_WARN(this->get_logger(), "connect_arms=false，跳过 Marvin 连接（仅腰部调试模式）");
  } else {
    kinematics_ready_ = initialize_kinematics();
    if (!kinematics_ready_) {
      RCLCPP_ERROR(this->get_logger(), "运动学初始化失败，末端位姿将不发布");
    }
    if (!connect_robot()) {
      RCLCPP_ERROR(this->get_logger(), "Marvin 机器人连接失败，节点将继续运行但无法下发控制");
    }
    RCLCPP_INFO(this->get_logger(), "机械臂控制：左臂=%s，右臂=%s（关闭时仍上报反馈）",
                left_arm_control_enabled_ ? "开启" : "只读",
                right_arm_control_enabled_ ? "开启" : "只读");
  }

  setup_io_teleop_interfaces();
  start_waist_thread();

  control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", control_rate_hz_);
  const auto period_ms = static_cast<int64_t>(1000.0 / control_rate_hz_);
  state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&HcTjMarvinBridgeNode::publish_joint_states, this));

  RCLCPP_INFO(this->get_logger(),
              "[hc_tj] Marvin 双臂桥接已启动，config: %s", resolved_config.c_str());
  RCLCPP_INFO(this->get_logger(),
              "joint_states %.0f Hz；启动自动回零=%s，后续回零用 io_teleop_reset_robot",
              control_rate_hz_, auto_home_on_start_ ? "是" : "否");
  if (!waist_cfg_.enabled) {
    RCLCPP_INFO(this->get_logger(), "腰部集成关闭，joint_states 仅含 %zu 个双臂关节",
                arm_joint_count_);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "腰部集成开启，joint_states 含 %zu 个关节（双臂 %zu + 腰部 %zu）",
                published_joint_names_.size(), arm_joint_count_, waist_cfg_.joints.size());
    if (waist_control_enabled_) {
      RCLCPP_INFO(this->get_logger(),
                  "腰部控制开启：servoJ (action 30) -> redis %s:%d/%s，%.0fHz，"
                  "限速 %.1f deg/s，看门狗 %d ms",
                  waist_cfg_.redis_host.c_str(), waist_cfg_.redis_port,
                  waist_cfg_.redis_key_command.c_str(), waist_cfg_.command_rate_hz,
                  waist_cfg_.max_velocity_deg_s, waist_cfg_.servo_timeout_ms);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "腰部只读模式：从 redis %s:%d/%s 读取状态，不下发腰部命令",
                  waist_cfg_.redis_host.c_str(), waist_cfg_.redis_port,
                  waist_cfg_.redis_key_status.c_str());
    }
  }
}

HcTjMarvinBridgeNode::~HcTjMarvinBridgeNode() {
  stop_waist_thread();
  if (state_timer_) {
    state_timer_->cancel();
    state_timer_.reset();
  }
  if (robot_connected_) {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    OnRelease();
    robot_connected_ = false;
    RCLCPP_INFO(this->get_logger(), "Marvin SDK 连接已释放");
  }
}

void HcTjMarvinBridgeNode::load_config(const std::string& config_path) {
  const YAML::Node root = [&]() -> YAML::Node {
    try {
      return YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
      throw std::runtime_error("Failed to load hardware config: " + config_path +
                               " error: " + e.what());
    }
  }();

  if (root["robot_ip"] && root["robot_ip"].IsSequence() && root["robot_ip"].size() >= 4) {
    for (size_t i = 0; i < 4; ++i) {
      robot_ip_[i] = static_cast<unsigned char>(root["robot_ip"][i].as<int>());
    }
  }

  hw_control_mode_ =
      root["control_mode"] ? root["control_mode"].as<std::string>() : "position";
  joint_vel_ratio_ = root["joint_vel_ratio"] ? root["joint_vel_ratio"].as<int>() : 100;
  joint_acc_ratio_ = root["joint_acc_ratio"] ? root["joint_acc_ratio"].as<int>() : 100;
  control_rate_hz_ =
      root["control_rate_hz"] ? root["control_rate_hz"].as<double>() : 100.0;
  reset_interpolation_steps_ =
      root["reset_interpolation_steps"] ? root["reset_interpolation_steps"].as<int>() : 100;
  reset_interpolation_ms_ =
      root["reset_interpolation_ms"] ? root["reset_interpolation_ms"].as<int>() : 5;
  auto_home_on_start_ =
      root["auto_home_on_start"] ? root["auto_home_on_start"].as<bool>() : true;
  kinematics_config_path_ =
      root["kinematics_config_path"]
          ? root["kinematics_config_path"].as<std::string>()
          : "CommonConfig/ccs_m6_40.MvKDCfg";
  const std::filesystem::path kinematics_path(kinematics_config_path_);
  if (kinematics_path.is_relative()) {
    const char* marvin_sdk_path = std::getenv("MARVIN_SDK_PATH");
    if (marvin_sdk_path == nullptr || marvin_sdk_path[0] == '\0') {
      throw std::runtime_error(
          "MARVIN_SDK_PATH is required when kinematics_config_path is relative");
    }
    kinematics_config_path_ =
        (std::filesystem::path(marvin_sdk_path) / kinematics_path).lexically_normal().string();
  }

  all_joint_names_ = parse_string_list(root["all_joint_names"]);
  left_cfg_ = parse_arm_side(root["left_arm"]);
  right_cfg_ = parse_arm_side(root["right_arm"]);

  if (const YAML::Node waist = root["waist"]) {
    waist_cfg_.joints = parse_string_list(waist["joints"]);
    waist_cfg_.enabled =
        waist["enabled"] ? waist["enabled"].as<bool>() : !waist_cfg_.joints.empty();
    if (waist["redis_host"]) {
      waist_cfg_.redis_host = waist["redis_host"].as<std::string>();
    }
    if (waist["redis_port"]) {
      waist_cfg_.redis_port = waist["redis_port"].as<int>();
    }
    if (waist["redis_key_command"]) {
      waist_cfg_.redis_key_command = waist["redis_key_command"].as<std::string>();
    }
    if (waist["redis_key_status"]) {
      waist_cfg_.redis_key_status = waist["redis_key_status"].as<std::string>();
    }
    if (waist["command_rate_hz"]) {
      waist_cfg_.command_rate_hz = waist["command_rate_hz"].as<double>();
    }
    if (waist["max_velocity_deg_s"]) {
      waist_cfg_.max_velocity_deg_s = waist["max_velocity_deg_s"].as<double>();
    }
    if (waist["servo_timeout_ms"]) {
      waist_cfg_.servo_timeout_ms = waist["servo_timeout_ms"].as<int>();
    }
    if (waist["cmd_timeout_sec"]) {
      waist_cfg_.cmd_timeout_sec = waist["cmd_timeout_sec"].as<double>();
    }
    if (waist["state_timeout_sec"]) {
      waist_cfg_.state_timeout_sec = waist["state_timeout_sec"].as<double>();
    }
    if (waist["status_in_degrees"]) {
      waist_cfg_.status_in_degrees = waist["status_in_degrees"].as<bool>();
    }
    waist_cfg_.limit_lower_deg = parse_double_list(waist["joint_limits_lower_deg"]);
    waist_cfg_.limit_upper_deg = parse_double_list(waist["joint_limits_upper_deg"]);
    for (const double rad : parse_double_list(waist["rest_j_pos"])) {
      waist_cfg_.rest_j_pos_deg.push_back(rad * kRad2Deg);
    }
    if (waist["home_timeout_sec"]) {
      waist_cfg_.home_timeout_sec = waist["home_timeout_sec"].as<double>();
    }
    if (waist["home_tolerance_deg"]) {
      waist_cfg_.home_tolerance_deg = waist["home_tolerance_deg"].as<double>();
    }
  }
  if (waist_cfg_.joints.empty()) {
    waist_cfg_.enabled = false;
  }

  waist_state_rad_.assign(waist_cfg_.joints.size(), 0.0);
  waist_target_deg_.assign(waist_cfg_.joints.size(), 0.0);
}

// action 30 的参数越界时，控制进程会对**每一条**指令抛异常，而且只在自己的 stderr
// 打一句 JSON parse error，现场极难定位。所以宁可启动即失败。
void HcTjMarvinBridgeNode::validate_waist_config() const {
  if (!waist_cfg_.enabled) {
    return;
  }
  if (!(waist_cfg_.max_velocity_deg_s > 0.0 &&
        waist_cfg_.max_velocity_deg_s <= kServoJMaxVelocityUpper)) {
    throw std::runtime_error(
        "waist.max_velocity_deg_s 必须在 (0, " + std::to_string(kServoJMaxVelocityUpper) +
        "] 内（SDK action 30 限制），当前 " +
        std::to_string(waist_cfg_.max_velocity_deg_s));
  }
  if (waist_cfg_.servo_timeout_ms < kServoJTimeoutMsLower ||
      waist_cfg_.servo_timeout_ms > kServoJTimeoutMsUpper) {
    throw std::runtime_error(
        "waist.servo_timeout_ms 必须在 [" + std::to_string(kServoJTimeoutMsLower) + ", " +
        std::to_string(kServoJTimeoutMsUpper) + "] 内（SDK action 30 限制），当前 " +
        std::to_string(waist_cfg_.servo_timeout_ms));
  }
  const size_t n = waist_cfg_.joints.size();
  if (waist_cfg_.limit_lower_deg.size() != n || waist_cfg_.limit_upper_deg.size() != n) {
    throw std::runtime_error("waist.joint_limits_*_deg 长度必须与 waist.joints 一致");
  }
  if (!waist_cfg_.rest_j_pos_deg.empty() && waist_cfg_.rest_j_pos_deg.size() != n) {
    throw std::runtime_error("waist.rest_j_pos 长度必须与 waist.joints 一致");
  }
  for (size_t i = 0; i < waist_cfg_.rest_j_pos_deg.size(); ++i) {
    const double v = waist_cfg_.rest_j_pos_deg[i];
    if (v < waist_cfg_.limit_lower_deg[i] || v > waist_cfg_.limit_upper_deg[i]) {
      throw std::runtime_error("waist.rest_j_pos 的 " + waist_cfg_.joints[i] + " = " +
                               std::to_string(v) + "° 超出限位 [" +
                               std::to_string(waist_cfg_.limit_lower_deg[i]) + ", " +
                               std::to_string(waist_cfg_.limit_upper_deg[i]) + "]");
    }
  }
  if (waist_cfg_.command_rate_hz <= 0.0) {
    throw std::runtime_error("waist.command_rate_hz 必须为正");
  }
  const double period_ms = 1000.0 / waist_cfg_.command_rate_hz;
  if (period_ms * 3.0 > waist_cfg_.servo_timeout_ms) {
    RCLCPP_WARN(this->get_logger(),
                "腰部下发周期 %.1fms 相对看门狗 %dms 余量不足，调度抖动可能误触发伺服失活",
                period_ms, waist_cfg_.servo_timeout_ms);
  }
}

void HcTjMarvinBridgeNode::build_joint_name_map() {
  joint_name_map_.clear();
  const std::vector<std::pair<std::string, const ArmSideConfig*>> sides = {
      {"left", &left_cfg_}, {"right", &right_cfg_}};
  for (const auto& [side, cfg_ptr] : sides) {
    for (size_t i = 0; i < cfg_ptr->arm_joints.size(); ++i) {
      joint_name_map_[cfg_ptr->arm_joints[i]] = {side, static_cast<int>(i)};
    }
  }
  waist_joint_set_.clear();
  waist_joint_set_.insert(waist_cfg_.joints.begin(), waist_cfg_.joints.end());
}

bool HcTjMarvinBridgeNode::initialize_kinematics() {
  FX_INT32L type[2]{};
  FX_DOUBLE gravity[2][3]{};
  FX_DOUBLE dh[2][8][4]{};
  FX_DOUBLE pnva[2][7][4]{};
  FX_DOUBLE boundary[2][4][3]{};
  FX_DOUBLE mass[2][7]{};
  FX_DOUBLE mass_center[2][7][3]{};
  FX_DOUBLE inertia[2][7][6]{};

  std::vector<char> config_path(kinematics_config_path_.begin(),
                                kinematics_config_path_.end());
  config_path.push_back('\0');
  if (LOADMvCfg(config_path.data(), type, gravity, dh, pnva, boundary, mass,
                mass_center, inertia) != FX_TRUE) {
    RCLCPP_ERROR(this->get_logger(), "无法加载运动学配置: %s",
                 kinematics_config_path_.c_str());
    return false;
  }

  for (int arm = 0; arm < 2; ++arm) {
    if (FX_Robot_Init_Type(arm, type[arm]) != FX_TRUE ||
        FX_Robot_Init_Kine(arm, dh[arm]) != FX_TRUE ||
        FX_Robot_Init_Lmt(arm, pnva[arm], boundary[arm]) != FX_TRUE) {
      RCLCPP_ERROR(this->get_logger(), "SDK %c 臂运动学参数初始化失败",
                   arm == 0 ? 'A' : 'B');
      return false;
    }
    FX_Robot_Tool_Rmv(arm);
  }

  // 厂商 SDK 默认会在每次正运动学时打印
  // [FxRobot - FX_Robot_Kine_Piolt]。状态发布以 100 Hz 对双臂各算一次，
  // 会造成约 200 行/秒的无意义刷屏；关闭日志不影响运动学计算。
  FX_LOG_SWITCH(0);

  RCLCPP_INFO(this->get_logger(), "双臂运动学已初始化: %s",
              kinematics_config_path_.c_str());
  return true;
}

bool HcTjMarvinBridgeNode::clear_arm_errors() {
  DCSS dcss;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    OnGetBuf(&dcss);
  }

  if (left_arm_control_enabled_ &&
      (dcss.m_State[0].m_ERRCode != 0 || dcss.m_State[0].m_CurState == 100)) {
    RCLCPP_WARN(this->get_logger(), "清除 A 臂错误 (ERRCode=%d)", dcss.m_State[0].m_ERRCode);
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    OnClearSet();
    OnClearErr_A();
    OnSetSend();
    sleep_ms(200);
  }
  if (right_arm_control_enabled_ &&
      (dcss.m_State[1].m_ERRCode != 0 || dcss.m_State[1].m_CurState == 100)) {
    RCLCPP_WARN(this->get_logger(), "清除 B 臂错误 (ERRCode=%d)", dcss.m_State[1].m_ERRCode);
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    OnClearSet();
    OnClearErr_B();
    OnSetSend();
    sleep_ms(200);
  }
  return true;
}

bool HcTjMarvinBridgeNode::connect_robot() {
  OnLocalLogOff();

  const bool init = OnLinkTo(robot_ip_[0], robot_ip_[1], robot_ip_[2], robot_ip_[3]);
  if (!init) {
    RCLCPP_ERROR(this->get_logger(),
                 "无法连接到 Marvin 控制器 %d.%d.%d.%d（端口可能被占用）",
                 robot_ip_[0], robot_ip_[1], robot_ip_[2], robot_ip_[3]);
    return false;
  }

  sleep_ms(200);
  const bool any_arm_control =
      left_arm_control_enabled_ || right_arm_control_enabled_;
  if (any_arm_control) {
    clear_arm_errors();
  }

  // 控制器可能停留在上一次运行的模式。SDK 建链后立即切换，必须早于
  // UDP 数据通道等待和启动回零，避免错误模式下继续启动机器人。
  robot_connected_ = true;
  if (any_arm_control && !setup_control_mode()) {
    RCLCPP_ERROR(this->get_logger(), "Marvin 控制模式指令提交失败，终止启动");
    OnRelease();
    robot_connected_ = false;
    return false;
  }

  int motion_tag = 0;
  int frame_update = 0;
  DCSS dcss;
  for (int i = 0; i < 10; ++i) {
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      OnGetBuf(&dcss);
    }
    if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
        frame_update != dcss.m_Out[0].m_OutFrameSerial) {
      motion_tag++;
      frame_update = dcss.m_Out[0].m_OutFrameSerial;
    }
    sleep_ms(100);
  }

  if (motion_tag == 0) {
    RCLCPP_ERROR(this->get_logger(), "Marvin UDP 数据通道未就绪，请检查网络与防火墙");
    OnRelease();
    robot_connected_ = false;
    return false;
  }

  if (any_arm_control && !verify_control_mode()) {
    RCLCPP_ERROR(this->get_logger(), "Marvin 控制模式反馈验证失败，禁止回零和外部控制");
    OnRelease();
    robot_connected_ = false;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Marvin 机器人连接成功 (%d.%d.%d.%d)",
              robot_ip_[0], robot_ip_[1], robot_ip_[2], robot_ip_[3]);
  return true;
}

bool HcTjMarvinBridgeNode::setup_control_mode() {
  if (!robot_connected_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(sdk_mutex_);
  bool ok = OnClearSet();
  if (hw_control_mode_ == "impedance") {
    double k[7] = {8.0, 8.0, 7.0, 6.0, 4.0, 3.0, 2.0};
    double d[7] = {0.25, 0.25, 0.25, 0.20, 0.15, 0.12, 0.10};
    if (left_arm_control_enabled_) {
      ok = OnSetTargetState_A(ARM_STATE_TORQ) && ok;
      ok = OnSetImpType_A(ARM_IMP_JOINT) && ok;
      ok = OnSetJointKD_A(k, d) && ok;
      ok = OnSetJointLmt_A(joint_vel_ratio_, joint_acc_ratio_) && ok;
    }
    if (right_arm_control_enabled_) {
      ok = OnSetTargetState_B(ARM_STATE_TORQ) && ok;
      ok = OnSetImpType_B(ARM_IMP_JOINT) && ok;
      ok = OnSetJointKD_B(k, d) && ok;
      ok = OnSetJointLmt_B(joint_vel_ratio_, joint_acc_ratio_) && ok;
    }
    RCLCPP_INFO(this->get_logger(), "已提交启用机械臂的关节阻抗模式，速度/加速度 %d%%",
                joint_vel_ratio_);
  } else {
    if (left_arm_control_enabled_) {
      ok = OnSetTargetState_A(ARM_STATE_POSITION) && ok;
      ok = OnSetJointLmt_A(joint_vel_ratio_, joint_acc_ratio_) && ok;
    }
    if (right_arm_control_enabled_) {
      ok = OnSetTargetState_B(ARM_STATE_POSITION) && ok;
      ok = OnSetJointLmt_B(joint_vel_ratio_, joint_acc_ratio_) && ok;
    }
    RCLCPP_INFO(this->get_logger(), "已提交启用机械臂的位置跟随模式，速度/加速度 %d%%",
                joint_vel_ratio_);
  }
  ok = OnSetSend() && ok;
  return ok;
}

bool HcTjMarvinBridgeNode::verify_control_mode() {
  const bool impedance = hw_control_mode_ == "impedance";
  const int expected_state = impedance ? ARM_STATE_TORQ : ARM_STATE_POSITION;
  for (int attempt = 0; attempt < kArmModeVerifyAttempts; ++attempt) {
    DCSS dcss;
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      OnGetBuf(&dcss);
    }
    const bool arm_a_ok = !left_arm_control_enabled_ ||
        (dcss.m_State[0].m_ERRCode == 0 &&
         dcss.m_State[0].m_CurState == expected_state &&
         (!impedance || dcss.m_In[0].m_ImpType == ARM_IMP_JOINT));
    const bool arm_b_ok = !right_arm_control_enabled_ ||
        (dcss.m_State[1].m_ERRCode == 0 &&
         dcss.m_State[1].m_CurState == expected_state &&
         (!impedance || dcss.m_In[1].m_ImpType == ARM_IMP_JOINT));
    if (arm_a_ok && arm_b_ok) {
      RCLCPP_INFO(this->get_logger(), "启用机械臂的%s反馈验证通过（State=%d%s）",
                  impedance ? "关节阻抗模式" : "位置模式", expected_state,
                  impedance ? ", ImpType=1" : "");
      return true;
    }
    sleep_ms(kArmModeVerifyIntervalMs);
  }

  DCSS dcss;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    OnGetBuf(&dcss);
  }
  RCLCPP_ERROR(this->get_logger(),
               "控制模式验证失败：A(ERR=%d, State=%d, ImpType=%d)，"
               "B(ERR=%d, State=%d, ImpType=%d)，期望 State=%d%s",
               dcss.m_State[0].m_ERRCode, dcss.m_State[0].m_CurState,
               dcss.m_In[0].m_ImpType, dcss.m_State[1].m_ERRCode,
               dcss.m_State[1].m_CurState, dcss.m_In[1].m_ImpType,
               expected_state, impedance ? ", ImpType=1" : "");
  return false;
}

void HcTjMarvinBridgeNode::publish_hardware_ready(bool ready) {
  hardware_ready_.store(ready);
  if (!hardware_ready_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = ready;
  hardware_ready_pub_->publish(msg);
}

void HcTjMarvinBridgeNode::setup_io_teleop_interfaces() {
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/io_teleop/joint_states", 10);
  ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/io_teleop/state_ee_poses", 10);
  ee_wrench_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/io_teleop/state_ee_wrenches", 10);
  joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/io_teleop/joint_cmd", 10,
      std::bind(&HcTjMarvinBridgeNode::joint_cmd_callback, this, std::placeholders::_1));
  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "io_teleop_reset_robot",
      std::bind(&HcTjMarvinBridgeNode::reset_service_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  rclcpp::QoS ready_qos(rclcpp::KeepLast(1));
  ready_qos.reliable();
  ready_qos.transient_local();
  hardware_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/io_teleop/hardware_ready", ready_qos);
  publish_hardware_ready(false);
}

void HcTjMarvinBridgeNode::publish_end_effector_feedback(
    const std::array<std::array<double, 7>, 2>& joint_positions_deg,
    const std::array<std::array<double, 6>, 2>& cartesian_wrenches) {
  const auto stamp = this->now();

  if (kinematics_ready_) {
    geometry_msgs::msg::PoseArray poses;
    poses.header.stamp = stamp;
    // PoseArray 顺序对应 controller_v2.yml: 右臂、左臂。每个位姿均相对
    // 厂商 SDK 中对应机械臂的基坐标系。
    poses.header.frame_id = "marvin_arm_base";
    poses.poses.reserve(2);

    for (const ArmSideConfig* cfg : {&right_cfg_, &left_cfg_}) {
      const int arm = sdk_arm_index(cfg->sdk_arm);
      FX_DOUBLE joints[7]{};
      for (size_t i = 0; i < 7; ++i) {
        joints[i] = joint_positions_deg[static_cast<size_t>(arm)][i];
      }
      Matrix4 matrix{};
      if (FX_Robot_Kine_FK(arm, joints, matrix) != FX_TRUE) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "SDK %c 臂正运动学计算失败", cfg->sdk_arm);
        return;
      }
      poses.poses.push_back(matrix_to_pose(matrix));
    }
    ee_pose_pub_->publish(poses);
  }

  std_msgs::msg::Float64MultiArray wrenches;
  wrenches.layout.dim.resize(2);
  wrenches.layout.dim[0].label = "arms:right,left";
  wrenches.layout.dim[0].size = 2;
  wrenches.layout.dim[0].stride = 12;
  wrenches.layout.dim[1].label = "wrench:Fx,Fy,Fz,Mx,My,Mz";
  wrenches.layout.dim[1].size = 6;
  wrenches.layout.dim[1].stride = 6;
  wrenches.data.reserve(12);
  for (const ArmSideConfig* cfg : {&right_cfg_, &left_cfg_}) {
    const int arm = sdk_arm_index(cfg->sdk_arm);
    for (size_t i = 0; i < 6; ++i) {
      wrenches.data.push_back(
          cartesian_wrenches[static_cast<size_t>(arm)][i]);
    }
  }
  ee_wrench_pub_->publish(wrenches);
}

// 腰部 Redis I/O 单独开线程，不能挂在 executor 上：一次 socket 超时就会把双臂
// joint_states 的 100Hz 循环一起拖停。servoJ 命令流本来也需要是条独立心跳。
void HcTjMarvinBridgeNode::start_waist_thread() {
  if (!waist_cfg_.enabled) {
    return;
  }
  try {
    sw::redis::ConnectionOptions opts;
    opts.host = waist_cfg_.redis_host;
    opts.port = waist_cfg_.redis_port;
    opts.socket_timeout = std::chrono::milliseconds(100);
    opts.connect_timeout = std::chrono::milliseconds(2000);
    waist_redis_ = std::make_unique<sw::redis::Redis>(opts);
    waist_redis_->ping();
    RCLCPP_INFO(this->get_logger(), "已连接腰部 Redis %s:%d", waist_cfg_.redis_host.c_str(),
                waist_cfg_.redis_port);
  } catch (const std::exception& e) {
    // redis++ 是惰性连接：客户端对象已建好，ping 失败只说明控制进程还没起，
    // 后续每拍都会自动重连，所以照常启动线程。
    RCLCPP_ERROR(this->get_logger(), "腰部 Redis 连接失败: %s（线程仍会启动并重试）",
                 e.what());
  }

  if (!waist_redis_) {
    RCLCPP_ERROR(this->get_logger(),
                 "腰部 Redis 客户端创建失败，腰部不可用，joint_states 将不会发布；"
                 "请检查 hardware.yaml 的 waist.redis_host/redis_port");
    return;
  }

  waist_thread_running_ = true;
  waist_task_id_ = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  waist_thread_ = std::thread([this]() { waist_thread_loop(); });
}

void HcTjMarvinBridgeNode::stop_waist_thread() {
  if (!waist_thread_running_.exchange(false)) {
    return;
  }
  if (waist_thread_.joinable()) {
    waist_thread_.join();
  }
  waist_redis_.reset();
}

void HcTjMarvinBridgeNode::waist_thread_loop() {
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / waist_cfg_.command_rate_hz));
  auto next = std::chrono::steady_clock::now();
  while (waist_thread_running_.load() && rclcpp::ok()) {
    next += period;
    waist_poll_status();
    if (waist_control_enabled_) {
      waist_send_servoj();
    }
    std::this_thread::sleep_until(next);
  }
}

void HcTjMarvinBridgeNode::waist_poll_status() {
  if (!waist_redis_) {
    return;
  }

  // 首帧之前 joint_states 完全发不出来，值得单独喊一嗓子指明根因；
  // 之后的断流由 publish_joint_states 的「已停更 X s」负责，不在这里重复告警。
  const bool have_first = [this]() {
    std::lock_guard<std::mutex> lock(waist_state_mutex_);
    return waist_state_received_;
  }();

  std::string raw;
  try {
    const auto value = waist_redis_->get(waist_cfg_.redis_key_status);
    if (!value) {
      if (!have_first) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "读不到 %s，joint_states 暂不发布；请确认腰部控制进程在运行",
                             waist_cfg_.redis_key_status.c_str());
      }
      return;
    }
    raw = *value;
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "读取 %s 失败: %s",
                         waist_cfg_.redis_key_status.c_str(), e.what());
    return;
  }

  std::vector<double> values;
  const auto data = nlohmann::json::parse(raw, nullptr, false);
  if (data.is_discarded() ||
      !extract_status_joints(data, waist_cfg_.joints.size(), values)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "%s 内容无法解析出 %zu 个腰部关节",
                         waist_cfg_.redis_key_status.c_str(), waist_cfg_.joints.size());
    return;
  }

  if (waist_cfg_.status_in_degrees) {
    for (auto& v : values) {
      v *= kDeg2Rad;
    }
  }

  std::lock_guard<std::mutex> lock(waist_state_mutex_);
  waist_state_rad_ = std::move(values);
  waist_state_stamp_ = std::chrono::steady_clock::now();
  if (!waist_state_received_) {
    waist_state_received_ = true;
    RCLCPP_INFO(this->get_logger(), "已读到腰部反馈首帧，joint_states 开始发布");
  }
}

void HcTjMarvinBridgeNode::waist_send_servoj() {
  if (!waist_control_enabled_) {
    return;
  }

  std::vector<double> target;
  if (waist_homing_.load()) {
    // 回零目标不走 joint_cmd 的断流判定，否则遥操作没在发时会被判成停发
    std::lock_guard<std::mutex> lock(waist_target_mutex_);
    target = waist_home_target_deg_;
  } else {
    std::lock_guard<std::mutex> lock(waist_target_mutex_);
    if (!waist_target_valid_) {
      return;
    }
    const double age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - waist_target_stamp_)
            .count();
    if (age > waist_cfg_.cmd_timeout_sec) {
      if (waist_streaming_) {
        waist_streaming_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "joint_cmd 腰部断流，停止下发；控制器 %d ms 看门狗将接管并保持位置",
                    waist_cfg_.servo_timeout_ms);
      }
      return;
    }
    if (!waist_streaming_) {
      waist_streaming_ = true;
      RCLCPP_INFO(this->get_logger(), "joint_cmd 腰部就绪，开始下发 servoJ 目标");
    }
    target = waist_target_deg_;
  }

  if (target.empty()) {
    return;
  }

  if (!waist_redis_) {
    return;
  }

  const nlohmann::json cmd = {
      {"action", kServoJAction},
      {"armID", kServoJArmId},
      {"params",
       {{"joint", clamp_waist_deg(target)},
        {"timestamp_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count()},
        {"max_velocity_deg_s", waist_cfg_.max_velocity_deg_s},
        {"timeout_ms", waist_cfg_.servo_timeout_ms}}},
      {"taskID", ++waist_task_id_}};

  try {
    waist_redis_->set(waist_cfg_.redis_key_command, cmd.dump());
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "发送 %s 失败: %s",
                         waist_cfg_.redis_key_command.c_str(), e.what());
  }
}

// action 30 的伺服线程直接调 sendJointPositions，控制器侧没有任何限位检查，
// 夹紧必须在这里做。
std::vector<double> HcTjMarvinBridgeNode::clamp_waist_deg(const std::vector<double>& deg) {
  std::vector<double> out(deg.size());
  std::string hits;
  for (size_t i = 0; i < deg.size(); ++i) {
    out[i] = std::max(waist_cfg_.limit_lower_deg[i],
                      std::min(waist_cfg_.limit_upper_deg[i], deg[i]));
    if (std::abs(out[i] - deg[i]) > 1e-6) {
      if (!hits.empty()) {
        hits += "; ";
      }
      hits += waist_cfg_.joints[i] + " " + std::to_string(deg[i]) + "° -> " +
              std::to_string(out[i]) + "°";
    }
  }
  if (!hits.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "joint_cmd 腰部超出控制器限位，已夹紧: %s", hits.c_str());
  }
  return out;
}

void HcTjMarvinBridgeNode::latch_waist_target(const sensor_msgs::msg::JointState& msg) {
  if (!waist_control_enabled_ || waist_homing_.load() || homing_in_progress_.load() || !hardware_ready_.load()) {
    return;
  }
  std::vector<double> values(waist_cfg_.joints.size(), 0.0);
  for (size_t i = 0; i < waist_cfg_.joints.size(); ++i) {
    const auto it = std::find(msg.name.begin(), msg.name.end(), waist_cfg_.joints[i]);
    if (it == msg.name.end()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "joint_cmd 缺少腰部关节 %s，整帧丢弃",
                           waist_cfg_.joints[i].c_str());
      return;
    }
    const double rad = msg.position[static_cast<size_t>(std::distance(msg.name.begin(), it))];
    if (!std::isfinite(rad)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "joint_cmd 中 %s 不是有限值，整帧丢弃",
                           waist_cfg_.joints[i].c_str());
      return;
    }
    values[i] = rad * kRad2Deg;
  }

  std::lock_guard<std::mutex> lock(waist_target_mutex_);
  waist_target_deg_ = std::move(values);
  waist_target_stamp_ = std::chrono::steady_clock::now();
  waist_target_valid_ = true;
}

void HcTjMarvinBridgeNode::publish_joint_states() {
  if (connect_arms_ && !robot_connected_) {
    return;
  }

  DCSS dcss;
  if (connect_arms_) {
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      OnGetBuf(&dcss);
    }

    if (!cmd_target_initialized_) {
      for (int i = 0; i < 7; ++i) {
        cmd_arm_deg_left_[static_cast<size_t>(i)] = dcss.m_Out[0].m_FB_Joint_Pos[i];
        cmd_arm_deg_right_[static_cast<size_t>(i)] = dcss.m_Out[1].m_FB_Joint_Pos[i];
      }
      cmd_target_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "已用 joint state 首帧初始化关节指令缓存");

      const bool any_control_enabled = left_arm_control_enabled_ ||
          right_arm_control_enabled_ || waist_control_enabled_;
      if (auto_home_on_start_ && any_control_enabled && !auto_home_done_) {
        auto_home_done_ = true;
        RCLCPP_INFO(this->get_logger(), "启动后自动回零至 rest_j_pos");
        std::thread([this]() { home_robot(); }).detach();
      }
    }
    std::array<std::array<double, 7>, 2> joint_positions_deg{};
    std::array<std::array<double, 6>, 2> cartesian_wrenches{};
    for (size_t arm = 0; arm < 2; ++arm) {
      for (size_t i = 0; i < 7; ++i) {
        joint_positions_deg[arm][i] = dcss.m_Out[arm].m_FB_Joint_Pos[i];
      }
      for (size_t i = 0; i < 6; ++i) {
        cartesian_wrenches[arm][i] = dcss.m_Out[arm].m_EST_Cart_FN[i];
      }
    }
    publish_end_effector_feedback(joint_positions_deg, cartesian_wrenches);
  }

  // 腰部关节必须齐全才能发布：TeleXperience 以 zhi_Link 为双臂 IK 基座，
  // 用错误的腰部位姿解算会直接把双臂指到错误位置。
  std::vector<double> waist_values;
  if (waist_cfg_.enabled) {
    std::lock_guard<std::mutex> lock(waist_state_mutex_);
    if (!waist_state_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "等待腰部反馈首帧，暂不发布 joint_states");
      return;
    }
    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                     waist_state_stamp_)
                           .count();
    if (age > waist_cfg_.state_timeout_sec) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "腰部反馈已停更 %.2f s，继续沿用最后一帧", age);
    }
    waist_values = waist_state_rad_;
  }

  // 仅腰模式下没有双臂首帧可挂靠，改用腰部首帧触发自动回零
  if (!connect_arms_ && waist_control_enabled_ && auto_home_on_start_ &&
      !auto_home_done_) {
    auto_home_done_ = true;
    RCLCPP_INFO(this->get_logger(), "启动后自动回零至 rest_j_pos（仅腰部）");
    std::thread([this]() { home_robot(); }).detach();
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.name = published_joint_names_;
  const size_t n = published_joint_names_.size();
  const size_t arm_count = arm_joint_count_;
  msg.position.resize(n, 0.0);
  msg.velocity.resize(n, 0.0);
  msg.effort.resize(n, 0.0);

  const size_t expected_waist_count = n - std::min(n, arm_count);
  if (waist_values.size() != expected_waist_count) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "joint_states 发布列表不一致：期望 %zu 个腰部值，实际 %zu 个；本帧丢弃",
        expected_waist_count, waist_values.size());
    return;
  }

  for (size_t i = 0; i < n; ++i) {
    if (i >= arm_count) {
      msg.position[i] = waist_values[i - arm_count];
      continue;
    }
    const auto it = joint_name_map_.find(all_joint_names_[i]);
    if (it == joint_name_map_.end()) {
      continue;
    }
    const bool is_left = it->second.first == "left";
    const int idx = it->second.second;
    if (idx < 0 || idx >= 7) {
      continue;
    }
    const size_t si = static_cast<size_t>(idx);
    const auto& out = is_left ? dcss.m_Out[0] : dcss.m_Out[1];
    msg.position[i] = out.m_FB_Joint_Pos[idx] * kDeg2Rad;
    msg.velocity[i] = out.m_FB_Joint_Vel[idx] * kDeg2Rad;
    msg.effort[i] = out.m_EST_Joint_Force[idx];
  }

  joint_state_pub_->publish(msg);
  const bool ready = (!connect_arms_ || (robot_connected_ && cmd_target_initialized_ && !homing_in_progress_.load())) &&
                     (!waist_control_enabled_ || (!waist_homing_.load() && waist_state_received_));
  publish_hardware_ready(ready);
}

void HcTjMarvinBridgeNode::joint_cmd_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->name.size() != msg->position.size() || msg->name.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "joint_cmd 关节名与位置数不匹配或为空");
    return;
  }

  // 1. HC_X1 回零期间丢弃全部目标指令（包括双臂和腰部）
  if (homing_in_progress_.load() || waist_homing_.load() || !hardware_ready_.load()) {
    return;
  }

  // 腰部由独立 Redis 驱动，回零完成后才锁存目标
  latch_waist_target(*msg);

  if (!connect_arms_ || !robot_connected_ ||
      (!left_arm_control_enabled_ && !right_arm_control_enabled_)) {
    return;
  }

  if (!cmd_target_initialized_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "关节指令缓冲未初始化，等待第一帧 joint state 反馈");
    return;
  }

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (waist_joint_set_.count(msg->name[i]) > 0) {
      continue;  // 腰部已在 latch_waist_target 里处理，走 Redis 而非 Marvin SDK
    }
    const auto it = joint_name_map_.find(msg->name[i]);
    if (it == joint_name_map_.end()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "收到未知关节名称: %s", msg->name[i].c_str());
      continue;
    }
    const std::string& side = it->second.first;
    const int idx = it->second.second;
    if (idx < 0 || idx >= 7) {
      continue;
    }
    const double angle_deg = msg->position[i] * kRad2Deg;
    if (side == "left" && left_arm_control_enabled_) {
      cmd_arm_deg_left_[static_cast<size_t>(idx)] = angle_deg;
    } else if (side == "right" && right_arm_control_enabled_) {
      cmd_arm_deg_right_[static_cast<size_t>(idx)] = angle_deg;
    }
  }

  double cmd_a[7]{};
  double cmd_b[7]{};
  for (int i = 0; i < 7; ++i) {
    cmd_a[i] = cmd_arm_deg_left_[static_cast<size_t>(i)];
    cmd_b[i] = cmd_arm_deg_right_[static_cast<size_t>(i)];
  }

  std::lock_guard<std::mutex> lock(sdk_mutex_);
  OnClearSet();
  if (left_arm_control_enabled_) {
    OnSetJointCmdPos_A(cmd_a);
  }
  if (right_arm_control_enabled_) {
    OnSetJointCmdPos_B(cmd_b);
  }
  OnSetSend();
}

// 腰部不像双臂那样由桥接插值：把目标交给控制器的 servoJ 伺服线程，
// 它会按 max_velocity_deg_s 限速逼近，这里只负责等它到位。
void HcTjMarvinBridgeNode::home_waist() {
  publish_hardware_ready(false);
  {
    std::lock_guard<std::mutex> lock(waist_target_mutex_);
    waist_home_target_deg_ = waist_cfg_.rest_j_pos_deg;
  }
  waist_homing_.store(true);

  RCLCPP_INFO(this->get_logger(), "腰部回零至 rest_j_pos，限速 %.1f deg/s",
              waist_cfg_.max_velocity_deg_s);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(waist_cfg_.home_timeout_sec));
  bool reached = false;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (waist_reached(waist_cfg_.rest_j_pos_deg, waist_cfg_.home_tolerance_deg)) {
      reached = true;
      break;
    }
    sleep_ms(20);
  }

  // 松开后若遥操作仍在发 joint_cmd，腰部会立刻回到跟随；没有的话由看门狗保持位置
  waist_homing_.store(false);
  {
    const bool ready = (!connect_arms_ || (robot_connected_ && cmd_target_initialized_ && !homing_in_progress_.load())) &&
                       (!waist_control_enabled_ || waist_state_received_);
    publish_hardware_ready(ready);
  }
  if (reached) {
    RCLCPP_INFO(this->get_logger(), "腰部回零完成");
  } else {
    RCLCPP_WARN(this->get_logger(),
                "腰部回零超时 %.1fs 仍未到位（容差 %.2f°），已停止等待；"
                "请检查腰部是否被卡住或 rest_j_pos 是否可达",
                waist_cfg_.home_timeout_sec, waist_cfg_.home_tolerance_deg);
  }
}

bool HcTjMarvinBridgeNode::waist_reached(const std::vector<double>& target_deg,
                                         double tolerance_deg) {
  std::lock_guard<std::mutex> lock(waist_state_mutex_);
  if (!waist_state_received_ || waist_state_rad_.size() != target_deg.size()) {
    return false;
  }
  for (size_t i = 0; i < target_deg.size(); ++i) {
    if (std::abs(waist_state_rad_[i] * kRad2Deg - target_deg[i]) > tolerance_deg) {
      return false;
    }
  }
  return true;
}

void HcTjMarvinBridgeNode::home_robot() {
  if (homing_in_progress_.exchange(true)) {
    RCLCPP_WARN(this->get_logger(), "回零正在进行中，忽略重复请求");
    return;
  }
  publish_hardware_ready(false);

  const bool home_left_arm =
      connect_arms_ && robot_connected_ && left_arm_control_enabled_;
  const bool home_right_arm =
      connect_arms_ && robot_connected_ && right_arm_control_enabled_;
  const bool home_arms = home_left_arm || home_right_arm;
  const bool home_waist_enabled =
      waist_control_enabled_ && !waist_cfg_.rest_j_pos_deg.empty();
  if (waist_control_enabled_ && waist_cfg_.rest_j_pos_deg.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "未配置 waist.rest_j_pos，腰部不参与回零；如需腰部回零请在 "
                "hardware.yaml 里补上（弧度，与 vr_configs.yml 的 folding_waist 一致）");
  }

  // 腰部先给目标再跑双臂插值，两者并行走，最后一起等腰部到位
  std::thread waist_thread;
  if (home_waist_enabled) {
    waist_thread = std::thread([this]() { home_waist(); });
  }

  if (!home_arms) {
    if (waist_thread.joinable()) {
      waist_thread.join();
    }
    homing_in_progress_.store(false);
    publish_hardware_ready(!waist_control_enabled_ || (!waist_homing_.load() && waist_state_received_));

    RCLCPP_INFO(this->get_logger(), "回零完成（双臂未连接，仅腰部）");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "开始回零（阻抗模式，插值 %d 步，每步 %d ms）...",
              reset_interpolation_steps_, reset_interpolation_ms_);

  double start_deg_l[7]{};
  double start_deg_r[7]{};
  double home_deg_l[7]{};
  double home_deg_r[7]{};
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    DCSS dcss;
    OnGetBuf(&dcss);
    for (int i = 0; i < 7; ++i) {
      start_deg_l[i] = dcss.m_Out[0].m_FB_Joint_Pos[i];
      start_deg_r[i] = dcss.m_Out[1].m_FB_Joint_Pos[i];
      home_deg_l[i] = left_cfg_.rest_j_pos[static_cast<size_t>(i)] * kRad2Deg;
      home_deg_r[i] = right_cfg_.rest_j_pos[static_cast<size_t>(i)] * kRad2Deg;
    }
  }

  const int steps = std::max(1, reset_interpolation_steps_);
  double cmd_l[7]{};
  double cmd_r[7]{};
  for (int s = 1; s <= steps && rclcpp::ok(); ++s) {
    const double ratio = static_cast<double>(s) / static_cast<double>(steps);
    for (int i = 0; i < 7; ++i) {
      cmd_l[i] = start_deg_l[i] + (home_deg_l[i] - start_deg_l[i]) * ratio;
      cmd_r[i] = start_deg_r[i] + (home_deg_r[i] - start_deg_r[i]) * ratio;
    }
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      OnClearSet();
      if (home_left_arm) {
        OnSetJointCmdPos_A(cmd_l);
      }
      if (home_right_arm) {
        OnSetJointCmdPos_B(cmd_r);
      }
      OnSetSend();
    }
    for (int i = 0; i < 7; ++i) {
      if (home_left_arm) {
        cmd_arm_deg_left_[static_cast<size_t>(i)] = cmd_l[i];
      }
      if (home_right_arm) {
        cmd_arm_deg_right_[static_cast<size_t>(i)] = cmd_r[i];
      }
    }
    sleep_ms(reset_interpolation_ms_);
  }

  // 腰部行程可能远长于双臂那 steps*ms，必须等它跑完再解除 homing
  if (waist_thread.joinable()) {
    waist_thread.join();
  }

  homing_in_progress_.store(false);
  RCLCPP_INFO(this->get_logger(), "回零完成");
}

void HcTjMarvinBridgeNode::reset_service_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  const bool arms_ready = connect_arms_ && robot_connected_ &&
      cmd_target_initialized_ &&
      (left_arm_control_enabled_ || right_arm_control_enabled_);
  const bool waist_ready =
      waist_control_enabled_ && !waist_cfg_.rest_j_pos_deg.empty();

  if (!arms_ready && !waist_ready) {
    response->success = false;
    if (connect_arms_ && !robot_connected_) {
      response->message = "Marvin 未连接，且腰部回零不可用，无法回零";
    } else if (connect_arms_ && !cmd_target_initialized_) {
      response->message = "Marvin 首帧反馈未就绪，且腰部回零不可用，无法回零";
    } else {
      response->message = "双臂与腰部均不可回零，请检查 waist.rest_j_pos 配置";
    }
    return;
  }

  RCLCPP_INFO(this->get_logger(), "收到 io_teleop_reset_robot 回零请求（双臂=%s，腰部=%s）",
              arms_ready ? "是" : "否", waist_ready ? "是" : "否");
  publish_hardware_ready(false);
  std::thread([this]() { home_robot(); }).detach();
  response->success = true;
  response->message = arms_ready && waist_ready ? "回零开始（双臂 + 腰部）"
                      : arms_ready              ? "回零开始（仅双臂）"
                                                : "回零开始（仅腰部）";
}

}  // namespace hc_tj_marvin_bridge
