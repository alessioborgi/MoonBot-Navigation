#include "rp_localizer/node.h"

Localizer2DNode::Localizer2DNode()
    : Node("localizer2d_node"), _tf_broadcaster(this) {
  // Declare the parameters
  this->declare_parameter<std::string>("laser_ns");
  this->declare_parameter<std::string>("base_link_ns");

  // Check if the parameters are set
  if (!this->has_parameter("laser_ns") ||
      !this->has_parameter("base_link_ns")) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Parameters 'laser_ns' and 'base_link_ns' are required but not set.");
    rclcpp::shutdown();
    throw std::runtime_error(
        "Parameters 'laser_ns' and 'base_link_ns' are required but not set.");
  }

  // Get the parameter values
  this->get_parameter("laser_ns", _laser_ns);
  this->get_parameter("base_link_ns", _base_link_ns);

  _map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&Localizer2DNode::mapCallback, this, std::placeholders::_1));
  _laser_offset_sub =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          _laser_ns + "/offset", 10,
          std::bind(&Localizer2DNode::laserOffsetCallback, this,
                    std::placeholders::_1));
  _scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      _laser_ns + "/scan", 10,
      std::bind(&Localizer2DNode::laserCallback, this, std::placeholders::_1));
  _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
      _base_link_ns + "/odom", 10);
  _pose_sub =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          _base_link_ns + "/initial_pose", 10,
          std::bind(&Localizer2DNode::poseCallback, this,
                    std::placeholders::_1));
}

void Localizer2DNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (_map_initialized) return;
  RCLCPP_INFO_ONCE(this->get_logger(), "Received map");
  auto& map = _localizer.getMap();
  map.loadFromOccupancyGrid(*msg);
  _localizer.setup();
  _map_initialized = true;
}

void Localizer2DNode::laserOffsetCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (msg->header.frame_id != _base_link_ns) return;

  std::lock_guard<std::mutex> lock(_laser_in_base_link_mutex);
  // RCLCPP_INFO(this->get_logger(), "Received laser offset");
  _laser_in_base_link.translation().x() = msg->pose.position.x;
  _laser_in_base_link.translation().y() = msg->pose.position.y;
  _laser_in_base_link.linear() =
      Eigen::Rotation2Df(
          2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w))
          .toRotationMatrix();
}

void Localizer2DNode::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!_pose_initialized || !_map_initialized) return;
  // RCLCPP_INFO(this->get_logger(), "Received laser scan");
  LaserScan scan;
  scan.fromROSMessage(*msg);
  const auto& laser_in_map = _localizer.update(scan);
  // RCLCPP_INFO(this->get_logger(), "Estimated pose: %f %f",
  //             pose.translation().x(), pose.translation().y());
  // const auto& pose = _localizer.pose();

  Eigen::Isometry2f base_link_in_map;
  {
    std::lock_guard<std::mutex> lock(_laser_in_base_link_mutex);
    base_link_in_map = laser_in_map * _laser_in_base_link.inverse();
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = _base_link_ns;
  odom.pose.pose.position.x = base_link_in_map.translation().x();
  odom.pose.pose.position.y = base_link_in_map.translation().y();
  odom.pose.pose.orientation.z = sin(
      atan2(base_link_in_map.linear()(1, 0), base_link_in_map.linear()(0, 0)) /
      2);
  odom.pose.pose.orientation.w = cos(
      atan2(base_link_in_map.linear()(1, 0), base_link_in_map.linear()(0, 0)) /
      2);
  _odom_pub->publish(odom);

  // Publish the transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = odom.header.stamp;
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = _base_link_ns;
  transform_stamped.transform.translation.x = odom.pose.pose.position.x;
  transform_stamped.transform.translation.y = odom.pose.pose.position.y;

  transform_stamped.transform.rotation.z = odom.pose.pose.orientation.z;
  transform_stamped.transform.rotation.w = odom.pose.pose.orientation.w;

  _tf_broadcaster.sendTransform(transform_stamped);
}

void Localizer2DNode::poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received initial pose");
  Eigen::Isometry2f pose;
  pose.translation().x() = msg->pose.pose.position.x;
  pose.translation().y() = msg->pose.pose.position.y;
  pose.linear() = Eigen::Rotation2Df(2 * atan2(msg->pose.pose.orientation.z,
                                               msg->pose.pose.orientation.w))
                      .toRotationMatrix();
  _localizer.setInitialGuess(pose);
  _pose_initialized = true;
}