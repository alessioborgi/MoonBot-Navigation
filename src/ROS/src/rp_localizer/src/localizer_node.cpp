#include <rclcpp/rclcpp.hpp>

#include "rp_localizer/localizer.h"
#include "rp_localizer/node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Localizer2DNode>());
  rclcpp::shutdown();
  return 0;
}