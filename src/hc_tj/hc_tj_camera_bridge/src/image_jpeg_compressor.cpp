#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageJpegCompressor : public rclcpp::Node
{
public:
  ImageJpegCompressor()
  : Node("image_jpeg_compressor")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/cameras/eye/color");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/io_teleop/camera_head/color");
    jpeg_quality_ = std::clamp(
      static_cast<int>(declare_parameter<int>("jpeg_quality", 80)), 1, 100);
    output_format_ = declare_parameter<std::string>("output_format", "jpeg");

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(3)).reliable().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(output_topic, output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&ImageJpegCompressor::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "camera bridge: %s -> %s (format=%s, quality=%d)",
      input_topic.c_str(), output_topic.c_str(), output_format_.c_str(), jpeg_quality_);
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      cv::Mat image;
      if (msg->encoding == "bgr8") {
        image = cv::Mat(
          static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
          const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
      } else if (msg->encoding == "rgb8") {
        const cv::Mat rgb(
          static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
          const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
        cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == "mono8") {
        image = cv::Mat(
          static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC1,
          const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Unsupported image encoding: %s",
          msg->encoding.c_str());
        return;
      }

      auto output = std::make_unique<sensor_msgs::msg::CompressedImage>();
      output->header = msg->header;
      output->format = output_format_;
      const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      if (!cv::imencode(".jpg", image, output->data, params)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "JPEG encoding returned no data");
        return;
      }
      publisher_->publish(std::move(output));
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "JPEG encoding failed: %s", error.what());
    }
  }

  int jpeg_quality_;
  std::string output_format_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageJpegCompressor>());
  rclcpp::shutdown();
  return 0;
}
