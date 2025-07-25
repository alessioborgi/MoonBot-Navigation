#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rp_planner/node.h"
#include "rp_planner/planner_dijkstra.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<DijkstraPlannerNode>());
  rclcpp::spin(std::make_shared<DijkstraPlannerNode>());
  rclcpp::shutdown();
  return 0;
}