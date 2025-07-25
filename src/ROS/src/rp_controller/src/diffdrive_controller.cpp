#include <rclcpp/rclcpp.hpp>

#include "rp_controller/node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialDriveControllerNode>());
  rclcpp::shutdown();
  return 0;
}