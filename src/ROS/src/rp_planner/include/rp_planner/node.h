#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rp_planner/planner_dijkstra.h"
#include "rp_planner/planner_dijkstra_v2.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class DijkstraPlannerNode : public rclcpp::Node {
 public:
  DijkstraPlannerNode();

 private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      _start_sub;
  PlannerDijkstra _planner;
  std::string _base_link_ns;
  bool _pose_initialized = false;
  bool _map_initialized = false;

  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
};

class DijkstraV2PlannerNode : public rclcpp::Node {
 public:
  DijkstraV2PlannerNode();

 protected:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      _start_sub;
  PlanningMapV2 _planner;
  std::string _base_link_ns;
  bool _pose_initialized = false;
  bool _map_initialized = false;

  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
};