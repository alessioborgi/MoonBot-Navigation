#include "rp_planner/node.h"
DijkstraPlannerNode::DijkstraPlannerNode() : Node("dijkstra_planner_node") {
  // Declare the parameter
  this->declare_parameter<std::string>("base_link_ns");
  if (!this->has_parameter("base_link_ns")) {
    RCLCPP_ERROR(this->get_logger(),
                 "Parameter 'base_link_ns' is required but not set.");
    rclcpp::shutdown();
    throw std::runtime_error(
        "Parameter 'base_link_ns' is required but not set.");
  }

  // Get the parameter value
  this->get_parameter("base_link_ns", _base_link_ns);

  _map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&DijkstraPlannerNode::map_callback, this,
                std::placeholders::_1));
  _path_pub =
      this->create_publisher<nav_msgs::msg::Path>(_base_link_ns + "/path", 10);

  // _start_sub = this->create_subscription<
  //     geometry_msgs::msg::PoseWithCovarianceStamped>(
  //     _base_link_ns + "/initial_pose", 10,
  //     std::bind(&DijkstraPlannerNode::start_callback, this,
  //               std::placeholders::_1));

  _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      _base_link_ns + "/goal_pose", 10,
      std::bind(&DijkstraPlannerNode::goal_callback, this,
                std::placeholders::_1));

  _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
}

void DijkstraPlannerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (_map_initialized) return;
  RCLCPP_INFO_ONCE(this->get_logger(), "Received map");
  auto& map = _planner.getMap();
  map.loadFromOccupancyGrid(*msg);
  _map_initialized = true;
}

void DijkstraPlannerNode::goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // if (!_pose_initialized || !_map_initialized) return;
  if (!_map_initialized) return;
  RCLCPP_INFO(this->get_logger(), "Received goal");

  geometry_msgs::msg::TransformStamped t;
  try {
    t = _tf_buffer->lookupTransform("map", _base_link_ns, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not get transform from map to %s: %s",
                _base_link_ns.c_str(), ex.what());
    return;
  }
  // Extract the start position
  Eigen::Vector2f start_position(t.transform.translation.x,
                                 t.transform.translation.y);
  _planner.setStart(start_position);

  const auto goal_position =
      Eigen::Vector2f(msg->pose.position.x, msg->pose.position.y);
  _planner.setGoal(goal_position);

  RCLCPP_INFO(this->get_logger(), "Planning path");
  // Attempt to plan a path and publish it
  std::vector<Eigen::Vector2f> path = _planner.plan();
  RCLCPP_INFO(this->get_logger(), "Path found with %ld points", path.size());
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  for (const auto& point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    path_msg.poses.push_back(pose);
  }
  _path_pub->publish(path_msg);
}

DijkstraV2PlannerNode::DijkstraV2PlannerNode()
    : Node("Dijkstrav2_planner_node") {
  // Declare the parameter
  this->declare_parameter<std::string>("base_link_ns");
  if (!this->has_parameter("base_link_ns")) {
    RCLCPP_ERROR(this->get_logger(),
                 "Parameter 'base_link_ns' is required but not set.");
    rclcpp::shutdown();
    throw std::runtime_error(
        "Parameter 'base_link_ns' is required but not set.");
  }

  // Get the parameter value
  this->get_parameter("base_link_ns", _base_link_ns);

  _map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&DijkstraV2PlannerNode::map_callback, this,
                std::placeholders::_1));
  _path_pub =
      this->create_publisher<nav_msgs::msg::Path>(_base_link_ns + "/path", 10);

  // _start_sub = this->create_subscription<
  //     geometry_msgs::msg::PoseWithCovarianceStamped>(
  //     _base_link_ns + "/initial_pose", 10,
  //     std::bind(&DijkstraPlannerNode::start_callback, this,
  //               std::placeholders::_1));

  _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      _base_link_ns + "/goal_pose", 10,
      std::bind(&DijkstraV2PlannerNode::goal_callback, this,
                std::placeholders::_1));

  _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
}

void DijkstraV2PlannerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (_map_initialized) return;
  RCLCPP_INFO_ONCE(this->get_logger(), "Received map");
  auto& map = _planner.getMap();
  map.loadFromOccupancyGrid(*msg);
  DistanceMap dmap;
  dmap.loadFromOccupancyGrid(*msg);
  _planner.loadFromGridMap(map, dmap);
  _map_initialized = true;
}

void DijkstraV2PlannerNode::goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // if (!_pose_initialized || !_map_initialized) return;
  if (!_map_initialized) return;
  RCLCPP_INFO(this->get_logger(), "Received goal");

  geometry_msgs::msg::TransformStamped t;
  try {
    t = _tf_buffer->lookupTransform("map", _base_link_ns, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not get transform from map to %s: %s",
                _base_link_ns.c_str(), ex.what());
    return;
  }
  // Extract the start position
  Eigen::Vector2f start_position(t.transform.translation.x,
                                 t.transform.translation.y);
  _planner.setStart(start_position);

  const auto goal_position =
      Eigen::Vector2f(msg->pose.position.x, msg->pose.position.y);
  _planner.setGoal(goal_position);
  _planner.computePolicy();

  RCLCPP_INFO(this->get_logger(), "Planning path");
  // Attempt to plan a path and publish it
  std::vector<Eigen::Vector2f> path = _planner.plan();
  RCLCPP_INFO(this->get_logger(), "Path found with %ld points", path.size());
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  for (const auto& point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    path_msg.poses.push_back(pose);
  }
  _path_pub->publish(path_msg);
}