#include "hc_tj_marvin_bridge_cpp/hc_tj_marvin_bridge_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hc_tj_marvin_bridge::HcTjMarvinBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
