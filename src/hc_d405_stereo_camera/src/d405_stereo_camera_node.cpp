#include <librealsense2/rs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

class D405StereoCameraNode : public rclcpp::Node
{
public:
  D405StereoCameraNode()
  : Node("d405_stereo_camera"), context_()
  {
    color_width_ = declare_parameter<int>("color.width", 640);
    color_height_ = declare_parameter<int>("color.height", 480);
    depth_width_ = declare_parameter<int>("depth.width", 640);
    depth_height_ = declare_parameter<int>("depth.height", 480);
    fps_ = declare_parameter<int>("fps", 30);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
    png_compression_ = declare_parameter<int>("png_compression", 3);
    publish_vr_composite_ = declare_parameter<bool>("publish_vr_composite", false);
    vr_output_topic_ = declare_parameter<std::string>(
      "vr_output_topic", "/io_teleop/camera_d405/vr/composite");
    vr_frame_id_ = declare_parameter<std::string>("vr_frame_id", "d405_stereo_vr");
    vr_max_pair_skew_ms_ = declare_parameter<double>("vr_max_pair_skew_ms", 50.0);
    swap_vr_views_ = declare_parameter<bool>("swap_vr_views", false);

    validate_parameters();

    left_ = make_camera("left", "260322275504", "/io_teleop/camera_d405_left", "d405_left");
    right_ = make_camera(
      "right", "260322276941", "/io_teleop/camera_d405_right", "d405_right");
    if (left_->enabled && right_->enabled && left_->serial_no == right_->serial_no) {
      throw std::invalid_argument("left.serial_no and right.serial_no must differ");
    }
    if (publish_vr_composite_ && (!left_->enabled || !right_->enabled)) {
      throw std::invalid_argument("VR composite requires both left and right D405 cameras enabled");
    }

    if (publish_vr_composite_) {
      auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
      vr_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        vr_output_topic_, image_qos);
    }

    if (left_->enabled) {
      start_pipeline(*left_);
    }
    if (right_->enabled) {
      start_pipeline(*right_);
    }

    running_.store(true);
    if (left_->enabled) {
      left_->thread = std::thread(&D405StereoCameraNode::capture_loop, this, left_.get());
    }
    if (right_->enabled) {
      right_->thread = std::thread(&D405StereoCameraNode::capture_loop, this, right_.get());
    }

    RCLCPP_INFO(
      get_logger(),
      "D405 configuration started: left=%s (%s -> %s), right=%s (%s -> %s), "
      "%dx%d color, %dx%d depth @ %d Hz",
      left_->enabled ? "enabled" : "disabled",
      left_->serial_no.c_str(), left_->topic_namespace.c_str(),
      right_->enabled ? "enabled" : "disabled",
      right_->serial_no.c_str(), right_->topic_namespace.c_str(),
      color_width_, color_height_, depth_width_, depth_height_, fps_);
    if (publish_vr_composite_) {
      RCLCPP_WARN(
        get_logger(),
        "VR side-by-side JPEG -> %s. Stop the old head camera bridge if it publishes the same topic.",
        vr_output_topic_.c_str());
    }
  }

  ~D405StereoCameraNode() override
  {
    running_.store(false);
    join_thread(left_.get());
    join_thread(right_.get());
    stop_pipeline(left_.get());
    stop_pipeline(right_.get());
  }

private:
  struct CameraState
  {
    explicit CameraState(rs2::context & context)
    : pipeline(context)
    {
    }

    std::string role;
    bool enabled{true};
    std::string serial_no;
    std::string topic_namespace;
    std::string frame_prefix;
    std::string color_frame_id;
    std::string depth_frame_id;
    rs2::pipeline pipeline;
    rs2::pipeline_profile profile;
    bool pipeline_started{false};
    float depth_scale_m{0.001F};
    std::chrono::steady_clock::time_point last_depth_scale_publish{};
    std::chrono::steady_clock::time_point last_calibration_publish{};
    sensor_msgs::msg::CameraInfo color_info;
    sensor_msgs::msg::CameraInfo depth_info;
    std_msgs::msg::String calibration_message;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr color_compressed_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr depth_compressed_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_scale_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calibration_publisher;
    std::thread thread;
    cv::Mat latest_color;
    rclcpp::Time latest_stamp{0, 0, RCL_ROS_TIME};
    std::uint64_t latest_sequence{0};
  };

  void validate_parameters() const
  {
    if (color_width_ <= 0 || color_height_ <= 0 || depth_width_ <= 0 ||
      depth_height_ <= 0 || fps_ <= 0)
    {
      throw std::invalid_argument("image dimensions and fps must be greater than zero");
    }
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      throw std::invalid_argument("jpeg_quality must be in [1, 100]");
    }
    if (png_compression_ < 0 || png_compression_ > 9) {
      throw std::invalid_argument("png_compression must be in [0, 9]");
    }
    if (!std::isfinite(vr_max_pair_skew_ms_) || vr_max_pair_skew_ms_ < 0.0) {
      throw std::invalid_argument("vr_max_pair_skew_ms must be finite and non-negative");
    }
    if (publish_vr_composite_ && (vr_output_topic_.empty() || vr_frame_id_.empty())) {
      throw std::invalid_argument("VR output topic and frame_id must not be empty");
    }
  }

  static std::string normalize_namespace(std::string value)
  {
    if (value.empty()) {
      throw std::invalid_argument("camera namespace must not be empty");
    }
    if (value.front() != '/') {
      value.insert(value.begin(), '/');
    }
    while (value.size() > 1 && value.back() == '/') {
      value.pop_back();
    }
    return value;
  }

  std::unique_ptr<CameraState> make_camera(
    const std::string & role, const std::string & default_serial,
    const std::string & default_namespace, const std::string & default_frame_prefix)
  {
    auto camera = std::make_unique<CameraState>(context_);
    camera->role = role;
    camera->enabled = declare_parameter<bool>(role + ".enabled", true);
    camera->serial_no = declare_parameter<std::string>(role + ".serial_no", default_serial);
    camera->topic_namespace = normalize_namespace(
      declare_parameter<std::string>(role + ".namespace", default_namespace));
    camera->frame_prefix = declare_parameter<std::string>(
      role + ".frame_prefix", default_frame_prefix);
    if (!camera->enabled) {
      RCLCPP_INFO(get_logger(), "%s D405 disabled by configuration", role.c_str());
      return camera;
    }
    if (camera->serial_no.empty() || camera->frame_prefix.empty()) {
      throw std::invalid_argument(role + " serial number/frame prefix must not be empty");
    }
    camera->color_frame_id = camera->frame_prefix + "_color_optical_frame";
    camera->depth_frame_id = camera->frame_prefix + "_depth_optical_frame";

    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    auto compressed_qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    camera->color_publisher = create_publisher<sensor_msgs::msg::Image>(
      camera->topic_namespace + "/color/image_raw", image_qos);
    camera->depth_publisher = create_publisher<sensor_msgs::msg::Image>(
      camera->topic_namespace + "/depth/image_raw", image_qos);
    camera->color_compressed_publisher =
      create_publisher<sensor_msgs::msg::CompressedImage>(
      camera->topic_namespace + "/color/compressed", compressed_qos);
    camera->depth_compressed_publisher =
      create_publisher<sensor_msgs::msg::CompressedImage>(
      camera->topic_namespace + "/depth/compressed", compressed_qos);
    camera->color_info_publisher = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera->topic_namespace + "/color/camera_info", info_qos);
    camera->depth_info_publisher = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera->topic_namespace + "/depth/camera_info", info_qos);
    camera->depth_scale_publisher = create_publisher<std_msgs::msg::Float32>(
      camera->topic_namespace + "/depth/scale", info_qos);
    camera->calibration_publisher = create_publisher<std_msgs::msg::String>(
      camera->topic_namespace + "/info", info_qos);
    return camera;
  }

  template<typename ContainerT>
  static void append_json_array(std::ostringstream & stream, const ContainerT & values)
  {
    stream << '[';
    bool first = true;
    for (const auto value : values) {
      if (!first) {
        stream << ',';
      }
      first = false;
      stream << value;
    }
    stream << ']';
  }

  static void append_camera_info_json(
    std::ostringstream & stream, const sensor_msgs::msg::CameraInfo & info)
  {
    stream << "{\"frame_id\":\"" << info.header.frame_id
           << "\",\"width\":" << info.width
           << ",\"height\":" << info.height
           << ",\"distortion_model\":\"" << info.distortion_model << "\",\"d\":";
    append_json_array(stream, info.d);
    stream << ",\"k\":";
    append_json_array(stream, info.k);
    stream << ",\"r\":";
    append_json_array(stream, info.r);
    stream << ",\"p\":";
    append_json_array(stream, info.p);
    stream << '}';
  }

  static std_msgs::msg::String make_calibration_message(const CameraState & camera)
  {
    std::ostringstream stream;
    stream << std::setprecision(17)
           << "{\"schema\":\"hc_d405_calibration/v1\",\"role\":\""
           << camera.role << "\",\"serial_number\":\"" << camera.serial_no
           << "\",\"depth_scale_m\":" << camera.depth_scale_m << ",\"color\":";
    append_camera_info_json(stream, camera.color_info);
    stream << ",\"depth\":";
    append_camera_info_json(stream, camera.depth_info);
    stream << '}';
    std_msgs::msg::String message;
    message.data = stream.str();
    return message;
  }

  static sensor_msgs::msg::CameraInfo make_camera_info(
    const rs2::video_stream_profile & profile, const std::string & frame_id)
  {
    const rs2_intrinsics intrinsics = profile.get_intrinsics();
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id;
    info.width = static_cast<std::uint32_t>(intrinsics.width);
    info.height = static_cast<std::uint32_t>(intrinsics.height);
    info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info.d.assign(intrinsics.coeffs, intrinsics.coeffs + 5);
    info.k.fill(0.0);
    info.k[0] = intrinsics.fx;
    info.k[2] = intrinsics.ppx;
    info.k[4] = intrinsics.fy;
    info.k[5] = intrinsics.ppy;
    info.k[8] = 1.0;
    info.r.fill(0.0);
    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;
    info.p.fill(0.0);
    info.p[0] = intrinsics.fx;
    info.p[2] = intrinsics.ppx;
    info.p[5] = intrinsics.fy;
    info.p[6] = intrinsics.ppy;
    info.p[10] = 1.0;
    return info;
  }

  void start_pipeline(CameraState & camera)
  {
    rs2::config config;
    config.enable_device(camera.serial_no);
    config.enable_stream(
      RS2_STREAM_COLOR, color_width_, color_height_, RS2_FORMAT_BGR8, fps_);
    config.enable_stream(
      RS2_STREAM_DEPTH, depth_width_, depth_height_, RS2_FORMAT_Z16, fps_);
    camera.profile = camera.pipeline.start(config);
    camera.pipeline_started = true;

    const auto color_profile = camera.profile.get_stream(RS2_STREAM_COLOR).as<
      rs2::video_stream_profile>();
    const auto depth_profile = camera.profile.get_stream(RS2_STREAM_DEPTH).as<
      rs2::video_stream_profile>();
    camera.color_info = make_camera_info(color_profile, camera.color_frame_id);
    camera.depth_info = make_camera_info(depth_profile, camera.depth_frame_id);
    camera.depth_scale_m = camera.profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    camera.calibration_message = make_calibration_message(camera);
    publish_depth_scale(camera);
    publish_calibration(camera);

    RCLCPP_INFO(
      get_logger(), "%s D405 %s opened; depth_scale=%.9f m/unit",
      camera.role.c_str(), camera.serial_no.c_str(), camera.depth_scale_m);
  }

  void capture_loop(CameraState * camera)
  {
    while (running_.load() && rclcpp::ok()) {
      try {
        const rs2::frameset frames = camera->pipeline.wait_for_frames(1000);
        const rs2::video_frame color = frames.get_color_frame();
        const rs2::depth_frame depth = frames.get_depth_frame();
        if (!color || !depth) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "%s frameset is missing color or depth", camera->role.c_str());
          continue;
        }
        const rclcpp::Time stamp = now();
        publish_camera_frames(*camera, color, depth, stamp);
      } catch (const rs2::error & error) {
        if (running_.load()) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "%s RealSense error in %s(%s): %s", camera->role.c_str(),
            error.get_failed_function().c_str(), error.get_failed_args().c_str(), error.what());
        }
      } catch (const std::exception & error) {
        if (running_.load()) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "%s capture error: %s", camera->role.c_str(), error.what());
        }
      }
    }
  }

  static sensor_msgs::msg::Image make_image(
    const rs2::video_frame & frame, const rclcpp::Time & stamp,
    const std::string & frame_id, const std::string & encoding)
  {
    sensor_msgs::msg::Image message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.height = static_cast<std::uint32_t>(frame.get_height());
    message.width = static_cast<std::uint32_t>(frame.get_width());
    message.encoding = encoding;
    message.is_bigendian = false;
    message.step = static_cast<std::uint32_t>(frame.get_stride_in_bytes());
    const auto * begin = static_cast<const std::uint8_t *>(frame.get_data());
    message.data.assign(begin, begin + frame.get_data_size());
    return message;
  }

  sensor_msgs::msg::CompressedImage encode_color(
    const cv::Mat & color, const rclcpp::Time & stamp, const std::string & frame_id) const
  {
    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.format = "jpeg";
    const std::vector<int> parameters = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", color, message.data, parameters)) {
      throw std::runtime_error("OpenCV failed to encode color JPEG");
    }
    return message;
  }

  sensor_msgs::msg::CompressedImage encode_depth(
    const cv::Mat & depth, const rclcpp::Time & stamp, const std::string & frame_id) const
  {
    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.format = "16UC1; png";
    const std::vector<int> parameters = {cv::IMWRITE_PNG_COMPRESSION, png_compression_};
    if (!cv::imencode(".png", depth, message.data, parameters)) {
      throw std::runtime_error("OpenCV failed to encode depth PNG");
    }
    return message;
  }

  void publish_camera_frames(
    CameraState & camera, const rs2::video_frame & color,
    const rs2::depth_frame & depth, const rclcpp::Time & stamp)
  {
    const cv::Mat color_view(
      color.get_height(), color.get_width(), CV_8UC3,
      const_cast<void *>(color.get_data()), color.get_stride_in_bytes());
    const cv::Mat depth_view(
      depth.get_height(), depth.get_width(), CV_16UC1,
      const_cast<void *>(depth.get_data()), depth.get_stride_in_bytes());

    camera.color_publisher->publish(
      make_image(color, stamp, camera.color_frame_id, sensor_msgs::image_encodings::BGR8));
    camera.depth_publisher->publish(
      make_image(depth, stamp, camera.depth_frame_id, sensor_msgs::image_encodings::TYPE_16UC1));
    camera.color_compressed_publisher->publish(
      encode_color(color_view, stamp, camera.color_frame_id));
    camera.depth_compressed_publisher->publish(
      encode_depth(depth_view, stamp, camera.depth_frame_id));

    camera.color_info.header.stamp = stamp;
    camera.depth_info.header.stamp = stamp;
    camera.color_info_publisher->publish(camera.color_info);
    camera.depth_info_publisher->publish(camera.depth_info);

    if (std::chrono::steady_clock::now() - camera.last_depth_scale_publish >=
      std::chrono::milliseconds(500))
    {
      publish_depth_scale(camera);
    }
    if (std::chrono::steady_clock::now() - camera.last_calibration_publish >=
      std::chrono::milliseconds(500))
    {
      publish_calibration(camera);
    }

    if (publish_vr_composite_) {
      {
        std::lock_guard<std::mutex> lock(stereo_mutex_);
        camera.latest_color = color_view.clone();
        camera.latest_stamp = stamp;
        ++camera.latest_sequence;
      }
      publish_vr_composite();
    }
  }

  static void publish_depth_scale(CameraState & camera)
  {
    std_msgs::msg::Float32 depth_scale_message;
    depth_scale_message.data = camera.depth_scale_m;
    camera.depth_scale_publisher->publish(depth_scale_message);
    camera.last_depth_scale_publish = std::chrono::steady_clock::now();
  }

  static void publish_calibration(CameraState & camera)
  {
    camera.calibration_publisher->publish(camera.calibration_message);
    camera.last_calibration_publish = std::chrono::steady_clock::now();
  }

  void publish_vr_composite()
  {
    cv::Mat left_image;
    cv::Mat right_image;
    rclcpp::Time output_stamp(0, 0, get_clock()->get_clock_type());
    {
      std::lock_guard<std::mutex> lock(stereo_mutex_);
      if (left_->latest_sequence == 0 || right_->latest_sequence == 0 ||
        left_->latest_sequence <= last_vr_left_sequence_ ||
        right_->latest_sequence <= last_vr_right_sequence_)
      {
        return;
      }
      const double skew_ms = std::abs(
        (left_->latest_stamp - right_->latest_stamp).seconds() * 1000.0);
      if (skew_ms > vr_max_pair_skew_ms_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "VR pair skew %.1f ms exceeds %.1f ms", skew_ms, vr_max_pair_skew_ms_);
        return;
      }
      left_image = left_->latest_color.clone();
      right_image = right_->latest_color.clone();
      output_stamp = left_->latest_stamp > right_->latest_stamp ?
        left_->latest_stamp : right_->latest_stamp;
      last_vr_left_sequence_ = left_->latest_sequence;
      last_vr_right_sequence_ = right_->latest_sequence;
    }

    if (left_image.size() != right_image.size()) {
      cv::resize(right_image, right_image, left_image.size(), 0.0, 0.0, cv::INTER_AREA);
    }
    std::array<cv::Mat, 2> views = {left_image, right_image};
    if (swap_vr_views_) {
      std::swap(views[0], views[1]);
    }
    cv::Mat side_by_side;
    cv::hconcat(views.data(), views.size(), side_by_side);
    vr_publisher_->publish(encode_color(side_by_side, output_stamp, vr_frame_id_));
  }

  static void join_thread(CameraState * camera)
  {
    if (camera != nullptr && camera->thread.joinable()) {
      camera->thread.join();
    }
  }

  void stop_pipeline(CameraState * camera)
  {
    if (camera == nullptr || !camera->pipeline_started) {
      return;
    }
    try {
      camera->pipeline.stop();
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Failed to stop %s pipeline: %s", camera->role.c_str(), error.what());
    }
    camera->pipeline_started = false;
  }

  rs2::context context_;
  std::unique_ptr<CameraState> left_;
  std::unique_ptr<CameraState> right_;
  int color_width_;
  int color_height_;
  int depth_width_;
  int depth_height_;
  int fps_;
  int jpeg_quality_;
  int png_compression_;
  bool publish_vr_composite_;
  std::string vr_output_topic_;
  std::string vr_frame_id_;
  double vr_max_pair_skew_ms_;
  bool swap_vr_views_;
  std::atomic<bool> running_{false};
  std::mutex stereo_mutex_;
  std::uint64_t last_vr_left_sequence_{0};
  std::uint64_t last_vr_right_sequence_{0};
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr vr_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<D405StereoCameraNode>());
  } catch (const rs2::error & error) {
    std::fprintf(
      stderr, "RealSense error in %s(%s): %s\n", error.get_failed_function().c_str(),
      error.get_failed_args().c_str(), error.what());
    result = 1;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "hc_d405_stereo_camera fatal error: %s\n", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
