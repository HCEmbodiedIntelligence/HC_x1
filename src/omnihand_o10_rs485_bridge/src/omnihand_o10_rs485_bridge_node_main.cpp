#include "omnihand_o10_rs485_bridge/omnihand_o10_rs485_bridge_node.hpp"

#include <iostream>
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<omnihand_o10_rs485_bridge::OmniHandO10Rs485BridgeNode>());
  } catch (const std::exception &e) {
    std::cerr << "Fatal: " << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
