#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class DepthPngCompressor : public rclcpp::Node
{
public:
  DepthPngCompressor()
  : Node("depth_png_compressor")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/cameras/eye/depth");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/io_teleop/camera_head/depth");
    png_compression_ = std::clamp(
      static_cast<int>(declare_parameter<int>("png_compression", 0)), 0, 9);

    // Depth is a live sensor stream.  Do not let a slow network subscriber
    // back-pressure capture/compression or queue stale frames.
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(output_topic, output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&DepthPngCompressor::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "depth PNG compressor: %s -> %s (compression=%d)",
      input_topic.c_str(), output_topic.c_str(), png_compression_);
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (msg->encoding != "16UC1" && msg->encoding != "mono16") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported depth encoding: %s (expected 16UC1 or mono16)",
        msg->encoding.c_str());
      return;
    }
    if (msg->is_bigendian != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Big-endian depth images are not supported");
      return;
    }
    const size_t minimum_size =
      static_cast<size_t>(msg->step) * static_cast<size_t>(msg->height);
    if (msg->data.size() < minimum_size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Depth payload is too short: got %zu, expected at least %zu",
        msg->data.size(), minimum_size);
      return;
    }

    try {
      const cv::Mat depth(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_16UC1,
        const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));

      auto output = std::make_unique<sensor_msgs::msg::CompressedImage>();
      output->header = msg->header;
      // This is a lossless, directly decodable 16-bit PNG. It intentionally
      // does not claim the compressedDepth transport wire format, which adds
      // an image_transport-specific binary header before the PNG bytes.
      output->format = msg->encoding + "; png";
      const std::vector<int> params{
        cv::IMWRITE_PNG_COMPRESSION, png_compression_};
      if (!cv::imencode(".png", depth, output->data, params)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "PNG encoding returned no data");
        return;
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Depth PNG: %ux%u, raw=%zu bytes, compressed=%zu bytes (%.1f%%)",
        msg->width, msg->height, msg->data.size(), output->data.size(),
        100.0 * static_cast<double>(output->data.size()) /
        static_cast<double>(msg->data.size()));
      publisher_->publish(std::move(output));
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Depth PNG encoding failed: %s",
        error.what());
    }
  }

  int png_compression_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthPngCompressor>());
  rclcpp::shutdown();
  return 0;
}
