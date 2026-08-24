#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <camera_msgs/msg/camera_parameters.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>

class CameraCalibrationJsonBridge : public rclcpp::Node
{
public:
  CameraCalibrationJsonBridge()
  : Node("camera_calibration_json_bridge")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/cameras/eye/param");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/io_teleop/camera_head/calibration");
    schema_ = declare_parameter<std::string>(
      "schema", "hc_camera_calibration/v1");
    robot_extrinsics_topic_ = declare_parameter<std::string>(
      "robot_extrinsics_topic", "/io_teleop/camera_head/extrinsics");
    robot_extrinsics_schema_ = declare_parameter<std::string>(
      "robot_extrinsics_schema", "hc_robot_camera_extrinsics/v1");
    camera_name_ = declare_parameter<std::string>("camera_name", "eye");
    color_width_ = declare_parameter<int>("color_width", 640);
    color_height_ = declare_parameter<int>("color_height", 400);
    depth_width_ = declare_parameter<int>("depth_width", 640);
    depth_height_ = declare_parameter<int>("depth_height", 400);
    translation_unit_ = declare_parameter<std::string>(
      "translation_unit", "unspecified");
    extrinsics_direction_ = declare_parameter<std::string>(
      "extrinsics_direction", "unspecified");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 2.0);

    publish_robot_static_tf_ = declare_parameter<bool>("publish_robot_static_tf", true);
    robot_common_frame_ = declare_parameter<std::string>("robot_common_frame", "zhi_Link");
    right_arm_base_frame_ = declare_parameter<std::string>(
      "right_arm_base_frame", "R_arm_0_Link");
    left_arm_base_frame_ = declare_parameter<std::string>(
      "left_arm_base_frame", "L_arm_0_Link");
    head_camera_frame_ = declare_parameter<std::string>("head_camera_frame", "camera_head");
    depth_optical_frame_ = declare_parameter<std::string>(
      "depth_optical_frame", "eye_depth_optical");
    rgb_optical_frame_ = declare_parameter<std::string>(
      "rgb_optical_frame", "eye_rgb_optical");
    zhi_to_right_arm_base_xyz_rpy_ = declare_xyz_rpy_parameter(
      "zhi_to_right_arm_base_xyz_rpy",
      {0.0, 0.0243, 0.32, 1.5708, 0.0, 0.0});
    zhi_to_left_arm_base_xyz_rpy_ = declare_xyz_rpy_parameter(
      "zhi_to_left_arm_base_xyz_rpy",
      {0.0, 0.1363, 0.32, -1.5708, 0.0, 0.0});
    zhi_to_head_camera_xyz_rpy_ = declare_xyz_rpy_parameter(
      "zhi_to_head_camera_xyz_rpy",
      {0.11323, 0.100299, 0.62155, -2.0944, 0.0, -1.5708});
    head_to_depth_optical_xyz_rpy_ = declare_xyz_rpy_parameter(
      "head_to_depth_optical_xyz_rpy",
      {0.0, 0.0, 0.0, 0.0, 0.0, 3.14159265358979323846});

    if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and greater than zero");
    }
    if (color_width_ <= 0 || color_height_ <= 0 || depth_width_ <= 0 || depth_height_ <= 0) {
      throw std::invalid_argument("color/depth dimensions must be greater than zero");
    }
    validate_robot_extrinsics();

    if (publish_robot_static_tf_) {
      static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      publish_robot_static_transforms(nullptr);
    }

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<std_msgs::msg::String>(output_topic_, output_qos);
    robot_extrinsics_publisher_ =
      create_publisher<std_msgs::msg::String>(robot_extrinsics_topic_, output_qos);
    robot_extrinsics_payload_ = serialize_robot_extrinsics();
    publish_robot_extrinsics();

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    subscription_ = create_subscription<camera_msgs::msg::CameraParameters>(
      input_topic_, input_qos,
      std::bind(&CameraCalibrationJsonBridge::on_parameters, this, std::placeholders::_1));

    const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = create_wall_timer(
      publish_period, std::bind(&CameraCalibrationJsonBridge::publish_cached, this));

    RCLCPP_INFO(
      get_logger(),
      "camera calibration bridge: %s (CameraParameters, reliable/transient_local/1) -> "
      "%s (String JSON, %.2f Hz, reliable/transient_local/1)",
      input_topic_.c_str(), output_topic_.c_str(), publish_rate_hz);
    RCLCPP_INFO(
      get_logger(),
      "robot-camera extrinsics: %s (String JSON, %.2f Hz, reliable/transient_local/1)",
      robot_extrinsics_topic_.c_str(), publish_rate_hz);
    if (publish_robot_static_tf_) {
      RCLCPP_INFO(
        get_logger(),
        "static camera TF (assume head_1=head_2=0 rad): %s -> {%s, %s, %s}; "
        "%s -> %s; calibration adds %s -> %s",
        robot_common_frame_.c_str(), right_arm_base_frame_.c_str(),
        left_arm_base_frame_.c_str(), head_camera_frame_.c_str(),
        head_camera_frame_.c_str(), depth_optical_frame_.c_str(),
        depth_optical_frame_.c_str(), rgb_optical_frame_.c_str());
    }
  }

private:
  using CameraParameters = camera_msgs::msg::CameraParameters;
  using XyzRpy = std::array<double, 6>;

  XyzRpy declare_xyz_rpy_parameter(
    const std::string & name, const XyzRpy & default_value)
  {
    const std::vector<double> values = declare_parameter<std::vector<double>>(
      name, std::vector<double>(default_value.begin(), default_value.end()));
    if (values.size() != 6) {
      throw std::invalid_argument(name + " must contain [x, y, z, roll, pitch, yaw]");
    }

    XyzRpy result{};
    for (std::size_t index = 0; index < result.size(); ++index) {
      if (!std::isfinite(values[index])) {
        throw std::invalid_argument(name + " must contain only finite values");
      }
      result[index] = values[index];
    }
    return result;
  }

  void validate_robot_extrinsics() const
  {
    if (robot_extrinsics_topic_.empty() || robot_extrinsics_schema_.empty()) {
      throw std::invalid_argument("robot extrinsics topic/schema must not be empty");
    }
    if (robot_common_frame_.empty() || right_arm_base_frame_.empty() ||
      left_arm_base_frame_.empty() || head_camera_frame_.empty() ||
      depth_optical_frame_.empty() || rgb_optical_frame_.empty())
    {
      throw std::invalid_argument("static TF frame names must not be empty");
    }
    const std::array<std::string, 5> child_frames = {
      right_arm_base_frame_, left_arm_base_frame_, head_camera_frame_,
      depth_optical_frame_, rgb_optical_frame_};
    for (std::size_t first = 0; first < child_frames.size(); ++first) {
      for (std::size_t second = first + 1; second < child_frames.size(); ++second) {
        if (child_frames[first] == child_frames[second]) {
          throw std::invalid_argument("static TF child frame names must be unique");
        }
      }
    }
    if (robot_common_frame_ == right_arm_base_frame_ ||
      robot_common_frame_ == left_arm_base_frame_ ||
      robot_common_frame_ == head_camera_frame_ ||
      robot_common_frame_ == depth_optical_frame_ ||
      robot_common_frame_ == rgb_optical_frame_)
    {
      throw std::invalid_argument("static TF parent and child frame names must differ");
    }
    if (translation_scale_to_meters() <= 0.0) {
      throw std::invalid_argument(
              "translation_unit must be one of: m, cm, mm");
    }
    if (extrinsics_direction_ != "rgb_to_depth" &&
      extrinsics_direction_ != "depth_to_rgb")
    {
      throw std::invalid_argument(
              "extrinsics_direction must be rgb_to_depth or depth_to_rgb");
    }
  }

  double translation_scale_to_meters() const
  {
    if (translation_unit_ == "m") {
      return 1.0;
    }
    if (translation_unit_ == "cm") {
      return 0.01;
    }
    if (translation_unit_ == "mm") {
      return 0.001;
    }
    return -1.0;
  }

  geometry_msgs::msg::TransformStamped make_static_transform(
    const std::string & parent_frame, const std::string & child_frame,
    const XyzRpy & xyz_rpy) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = xyz_rpy[0];
    transform.transform.translation.y = xyz_rpy[1];
    transform.transform.translation.z = xyz_rpy[2];

    tf2::Quaternion quaternion;
    quaternion.setRPY(xyz_rpy[3], xyz_rpy[4], xyz_rpy[5]);
    quaternion.normalize();
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    return transform;
  }

  static tf2::Quaternion make_quaternion(const XyzRpy & xyz_rpy)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(xyz_rpy[3], xyz_rpy[4], xyz_rpy[5]);
    quaternion.normalize();
    return quaternion;
  }

  geometry_msgs::msg::TransformStamped make_rgb_optical_transform(
    const CameraParameters & parameters) const
  {
    tf2::Matrix3x3 rotation(
      parameters.rotation[0], parameters.rotation[1], parameters.rotation[2],
      parameters.rotation[3], parameters.rotation[4], parameters.rotation[5],
      parameters.rotation[6], parameters.rotation[7], parameters.rotation[8]);
    const double scale = translation_scale_to_meters();
    tf2::Transform raw(
      rotation,
      tf2::Vector3(
        parameters.translation[0] * scale,
        parameters.translation[1] * scale,
        parameters.translation[2] * scale));
    // rgb_to_depth means p_depth = R * p_rgb + t, i.e. the pose of the
    // RGB optical frame expressed in the depth optical parent frame.
    const tf2::Transform depth_to_rgb =
      extrinsics_direction_ == "rgb_to_depth" ? raw : raw.inverse();
    const tf2::Quaternion quaternion = depth_to_rgb.getRotation().normalized();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = depth_optical_frame_;
    transform.child_frame_id = rgb_optical_frame_;
    transform.transform.translation.x = depth_to_rgb.getOrigin().x();
    transform.transform.translation.y = depth_to_rgb.getOrigin().y();
    transform.transform.translation.z = depth_to_rgb.getOrigin().z();
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    return transform;
  }

  void publish_robot_static_transforms(const CameraParameters * parameters)
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms = {
      make_static_transform(
        robot_common_frame_, right_arm_base_frame_, zhi_to_right_arm_base_xyz_rpy_),
      make_static_transform(
        robot_common_frame_, left_arm_base_frame_, zhi_to_left_arm_base_xyz_rpy_),
      make_static_transform(
        robot_common_frame_, head_camera_frame_, zhi_to_head_camera_xyz_rpy_),
      make_static_transform(
        head_camera_frame_, depth_optical_frame_, head_to_depth_optical_xyz_rpy_)};
    if (parameters != nullptr) {
      transforms.push_back(make_rgb_optical_transform(*parameters));
    }
    static_tf_broadcaster_->sendTransform(transforms);
  }

  static void append_json_string(std::ostringstream & output, const std::string & value)
  {
    output << '"';
    for (const unsigned char character : value) {
      switch (character) {
        case '"':
          output << "\\\"";
          break;
        case '\\':
          output << "\\\\";
          break;
        case '\b':
          output << "\\b";
          break;
        case '\f':
          output << "\\f";
          break;
        case '\n':
          output << "\\n";
          break;
        case '\r':
          output << "\\r";
          break;
        case '\t':
          output << "\\t";
          break;
        default:
          if (character < 0x20) {
            output << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<int>(character) << std::dec << std::setfill(' ');
          } else {
            output << static_cast<char>(character);
          }
      }
    }
    output << '"';
  }

  static void append_number(std::ostringstream & output, double value, bool & all_finite)
  {
    if (std::isfinite(value)) {
      output << value;
    } else {
      output << "null";
      all_finite = false;
    }
  }

  void append_robot_transform(
    std::ostringstream & output, const std::string & parent_frame,
    const std::string & child_frame,
    const XyzRpy & xyz_rpy, bool & all_finite) const
  {
    const tf2::Quaternion quaternion = make_quaternion(xyz_rpy);
    output << "{\"parent_frame\":";
    append_json_string(output, parent_frame);
    output << ",\"child_frame\":";
    append_json_string(output, child_frame);
    output << ",\"translation_m\":[";
    append_number(output, xyz_rpy[0], all_finite);
    output << ',';
    append_number(output, xyz_rpy[1], all_finite);
    output << ',';
    append_number(output, xyz_rpy[2], all_finite);
    output << "],\"rotation_rpy_rad\":[";
    append_number(output, xyz_rpy[3], all_finite);
    output << ',';
    append_number(output, xyz_rpy[4], all_finite);
    output << ',';
    append_number(output, xyz_rpy[5], all_finite);
    output << "],\"rotation_xyzw\":[";
    append_number(output, quaternion.x(), all_finite);
    output << ',';
    append_number(output, quaternion.y(), all_finite);
    output << ',';
    append_number(output, quaternion.z(), all_finite);
    output << ',';
    append_number(output, quaternion.w(), all_finite);
    output << "]}";
  }

  std::string serialize_robot_extrinsics() const
  {
    bool all_finite = true;
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    output << "{\"schema\":";
    append_json_string(output, robot_extrinsics_schema_);
    output << ",\"source\":\"hc_tj_camera_bridge_static_transform_config\"";
    output << ",\"assumptions\":{\"head_1_rad\":0.0,\"head_2_rad\":0.0,";
    output << "\"camera_frame_is_mechanical_link\":true}";
    output << ",\"common_frame\":";
    append_json_string(output, robot_common_frame_);
    output << ",\"transforms\":[";
    append_robot_transform(
      output, robot_common_frame_, right_arm_base_frame_,
      zhi_to_right_arm_base_xyz_rpy_, all_finite);
    output << ',';
    append_robot_transform(
      output, robot_common_frame_, left_arm_base_frame_,
      zhi_to_left_arm_base_xyz_rpy_, all_finite);
    output << ',';
    append_robot_transform(
      output, robot_common_frame_, head_camera_frame_,
      zhi_to_head_camera_xyz_rpy_, all_finite);
    output << "],\"sensor_frames\":{\"mechanical_frame\":";
    append_json_string(output, head_camera_frame_);
    output << ",\"depth_optical_frame\":";
    append_json_string(output, depth_optical_frame_);
    output << ",\"rgb_optical_frame\":";
    append_json_string(output, rgb_optical_frame_);
    output << ",\"mechanical_to_depth_optical\":";
    append_robot_transform(
      output, head_camera_frame_, depth_optical_frame_,
      head_to_depth_optical_xyz_rpy_, all_finite);
    output << ",\"rgb_depth_calibration_topic\":";
    append_json_string(output, output_topic_);
    output << ",\"rgb_depth_direction\":";
    append_json_string(output, extrinsics_direction_);
    output << ",\"rgb_depth_translation_unit\":";
    append_json_string(output, translation_unit_);
    output << "}}";
    if (!all_finite) {
      throw std::invalid_argument("robot extrinsics contain non-finite values");
    }
    return output.str();
  }

  template<typename ArrayT>
  static void append_number_array(
    std::ostringstream & output, const ArrayT & values, bool & all_finite)
  {
    output << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index != 0) {
        output << ',';
      }
      append_number(output, values[index], all_finite);
    }
    output << ']';
  }

  std::pair<std::string, bool> serialize(const CameraParameters & parameters)
  {
    bool all_finite = true;
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<float>::max_digits10);

    output << "{\"schema\":";
    append_json_string(output, schema_);
    output << ",\"camera\":";
    append_json_string(output, camera_name_);
    output << ",\"source_topic\":";
    append_json_string(output, input_topic_);
    output << ",\"received_at_ros_time_ns\":" << get_clock()->now().nanoseconds();
    output << ",\"resolution\":{";
    output << "\"color\":{\"width\":" << color_width_
           << ",\"height\":" << color_height_ << "},";
    output << "\"depth\":{\"width\":" << depth_width_
           << ",\"height\":" << depth_height_ << "}}";

    output << ",\"rgb\":{\"fx\":";
    append_number(output, parameters.rgb_fx, all_finite);
    output << ",\"fy\":";
    append_number(output, parameters.rgb_fy, all_finite);
    output << ",\"cx\":";
    append_number(output, parameters.rgb_cx, all_finite);
    output << ",\"cy\":";
    append_number(output, parameters.rgb_cy, all_finite);
    output << ",\"distortion_order\":[\"k1\",\"k2\",\"p1\",\"p2\",\"k3\"],";
    output << "\"distortion_coefficients\":";
    append_number_array(output, parameters.rgb_distortion_coeffs, all_finite);
    output << '}';

    output << ",\"depth\":{\"fx\":";
    append_number(output, parameters.dep_fx, all_finite);
    output << ",\"fy\":";
    append_number(output, parameters.dep_fy, all_finite);
    output << ",\"cx\":";
    append_number(output, parameters.dep_cx, all_finite);
    output << ",\"cy\":";
    append_number(output, parameters.dep_cy, all_finite);
    output << ",\"distortion_order\":[\"k1\",\"k2\",\"p1\",\"p2\",\"k3\"],";
    output << "\"distortion_coefficients\":";
    append_number_array(output, parameters.dep_distortion_coeffs, all_finite);
    output << '}';

    output << ",\"rgb_depth_extrinsics\":{\"rotation_layout\":\"row_major\",";
    output << "\"rotation\":";
    append_number_array(output, parameters.rotation, all_finite);
    output << ",\"translation\":";
    append_number_array(output, parameters.translation, all_finite);
    output << ",\"translation_unit\":";
    append_json_string(output, translation_unit_);
    output << ",\"direction\":";
    append_json_string(output, extrinsics_direction_);
    output << "}}";

    return {output.str(), all_finite};
  }

  void on_parameters(const CameraParameters::ConstSharedPtr parameters)
  {
    auto serialized = serialize(*parameters);
    {
      std::lock_guard<std::mutex> lock(payload_mutex_);
      cached_payload_ = serialized.first;
    }

    publish(serialized.first);
    if (publish_robot_static_tf_) {
      publish_robot_static_transforms(parameters.get());
    }
    RCLCPP_INFO(
      get_logger(), "Received and cached camera calibration (%zu JSON bytes)",
      serialized.first.size());
    if (!serialized.second) {
      RCLCPP_WARN(
        get_logger(),
        "Camera calibration contains non-finite values; they were encoded as JSON null");
    }
  }

  void publish(const std::string & payload)
  {
    std_msgs::msg::String output;
    output.data = payload;
    publisher_->publish(output);
  }

  void publish_robot_extrinsics()
  {
    std_msgs::msg::String output;
    output.data = robot_extrinsics_payload_;
    robot_extrinsics_publisher_->publish(output);
  }

  void publish_cached()
  {
    std::string payload;
    {
      std::lock_guard<std::mutex> lock(payload_mutex_);
      payload = cached_payload_;
    }
    if (!payload.empty()) {
      publish(payload);
    }
    publish_robot_extrinsics();
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string schema_;
  std::string robot_extrinsics_topic_;
  std::string robot_extrinsics_schema_;
  std::string camera_name_;
  std::string translation_unit_;
  std::string extrinsics_direction_;
  bool publish_robot_static_tf_;
  std::string robot_common_frame_;
  std::string right_arm_base_frame_;
  std::string left_arm_base_frame_;
  std::string head_camera_frame_;
  std::string depth_optical_frame_;
  std::string rgb_optical_frame_;
  XyzRpy zhi_to_right_arm_base_xyz_rpy_{};
  XyzRpy zhi_to_left_arm_base_xyz_rpy_{};
  XyzRpy zhi_to_head_camera_xyz_rpy_{};
  XyzRpy head_to_depth_optical_xyz_rpy_{};
  int color_width_;
  int color_height_;
  int depth_width_;
  int depth_height_;
  std::mutex payload_mutex_;
  std::string cached_payload_;
  std::string robot_extrinsics_payload_;
  rclcpp::Subscription<CameraParameters>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_extrinsics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraCalibrationJsonBridge>());
  rclcpp::shutdown();
  return 0;
}
