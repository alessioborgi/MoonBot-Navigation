#include "rp_map_server/node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapServerNode>());
  rclcpp::shutdown();
  return 0;
}