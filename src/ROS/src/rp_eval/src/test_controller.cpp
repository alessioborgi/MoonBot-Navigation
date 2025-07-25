#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rp_commons/grid_map.h"
#include "rp_controller/node.h"
#include "rp_map_server/node.h"
#include "rp_planner/node.h"
#include "rp_simulator/laser_scanner.h"
#include "rp_simulator/unicycle.h"
#include "rp_simulator/world.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Initialize simulator
  auto node = std::make_shared<rclcpp::Node>("eval_node");
  GridMap grid_map;
  grid_map.loadFromImage("assets/diag_small.png", 0.1);
  auto world = std::make_shared<World>(grid_map, node, 0.1);
  // Create initial pose
  Eigen::Quaternionf q(0.7448586746707018, 0.0, 0.0, 0.6672222678896483);
  Eigen::Isometry2f pose = Eigen::Isometry2f::Identity();
  pose.translation() = Eigen::Vector2f(4.5, -15);
  pose.linear() = q.normalized().toRotationMatrix().topLeftCorner<2, 2>();

  auto robot =
      std::make_shared<UnicyclePlatform>(*world, "robot_1", node, pose, true);
  robot->_radius = 0.2;
  Eigen::Isometry2f laser_pose = Eigen::Isometry2f::Identity();
  auto laser_scanner =
      std::make_shared<LaserScanner>(*robot, "laser_1", node, laser_pose, 10.0);
  laser_scanner->_radius = 0.2;

  auto map_server = std::make_shared<MapServerNode>();
  auto planner = std::make_shared<DijkstraV2PlannerNode>();
  auto controller = std::make_shared<DifferentialDriveControllerNode>();

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(
          "robot_1/goal_pose", 10);
  auto goal_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  goal_msg->pose.position.x = 4.5;
  goal_msg->pose.position.y = -20;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(map_server);
  executor.add_node(planner);
  executor.add_node(controller);
  executor.add_node(node);
  uint64_t it = 0;
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
  while (rclcpp::ok()) {
    if (it == 30000) {
      goal_pub->publish(*goal_msg);
    }
    it++;
    executor.spin_some();
    const auto goal_dist =
        (robot->globalPose().translation() -
         Eigen::Vector2f(goal_msg->pose.position.x, goal_msg->pose.position.y))
            .norm();
    if (goal_dist < 1.5) {
      RCLCPP_INFO(node->get_logger(), "Goal reached!");
      return 0;
    }
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    if (elapsed_seconds.count() > 60) {
      RCLCPP_INFO(node->get_logger(), "Timeout!");
      return -1;
    }
  }
  rclcpp::shutdown();

  return 0;
}