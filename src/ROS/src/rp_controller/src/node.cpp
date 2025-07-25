#include "rp_controller/node.h"

#include <Eigen/Dense>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rp_controller/differential_drive_controller.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <mutex>

DifferentialDriveControllerNode::DifferentialDriveControllerNode()
    : Node("differential_drive_controller_node") {
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

  _controller = std::make_shared<DiffDriveController>();
  _timer = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DifferentialDriveControllerNode::update, this));

  _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  // TODO: Create the publishers and subscribers
  _twist_pub = this->create_publisher<geometry_msgs::msg::Twist>(_base_link_ns + "/cmd_vel", 10);
  _path_sub = this->create_subscription<nav_msgs::msg::Path>(
      _base_link_ns + "/path", 10, std::bind(&DifferentialDriveControllerNode::pathCallback, this, std::placeholders::_1));
  _scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      _laser_ns + "/scan", 10, std::bind(&DifferentialDriveControllerNode::laserCallback, this, std::placeholders::_1));
}

void DifferentialDriveControllerNode::pathCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  // Convert the nav_msgs::Path to a vector of Eigen::Isometry2f
  std::vector<Eigen::Isometry2f> waypoints;
  for (const auto& pose : msg->poses) {
    
    // TODO: Read the pose quaternion and position
    auto position = pose.pose.position;
    auto orientation = pose.pose.orientation;
    // auto or_msg = msg->poses[0].pose.orientation;
    // Eigen::Quaternionf orientation(or_msg.w, or_msg.x, or_msg.y, or_msg.z);
    std::cout << pose.pose.position.x << " " << pose.pose.position.y << " _ " << orientation.w << " " << orientation.x << " " << orientation.y << " " << orientation.z << " " << std::endl;

    Eigen::Isometry2f waypoint = Eigen::Isometry2f::Identity();
    
    // TODO: Set the rotation and translation of the waypoint
    waypoint.translate(Eigen::Vector2f(position.x, position.y));
    waypoint.rotate(Eigen::Rotation2Df(2 * atan2(orientation.z, orientation.w))); 
    // waypoint.translation() = Eigen::Vector2f(position.x, position.y);
    // waypoint.linear() = orientation.normalized().toRotationMatrix().topLeftCorner<2, 2>();

    waypoints.push_back(waypoint);
  }
  RCLCPP_INFO(this->get_logger(), "Received %lu waypoints", waypoints.size());
  
  // TODO: Set the waypoints in the controller
  _controller->setWaypoints(waypoints);
}

void DifferentialDriveControllerNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(_measurements_mutex);
  
  LaserScan current_scan;
  current_scan.fromROSMessage(*msg);
  // std::vector<Eigen::Vector2f> measurements = // TODO: Convert the LaserScan to a vector of Eigen::Vector2f;
  std::vector<Eigen::Vector2f> measurements = current_scan.toCartesian();

  // TODO: Set the laser measurements in the controller
  _controller->setLaserMeasurements(measurements);
}

void DifferentialDriveControllerNode::update() {
  std::lock_guard<std::mutex> lock(_measurements_mutex);
  
  if (_controller->isDone()) {
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  try {
    t = _tf_buffer->lookupTransform("map", "robot_1", tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not get transform from map to robot_1: %s", ex.what());
    return;
  }
  // Convert TransformStamped to a Eigen::Isometry2f
  // TODO: Read the pose quaternion and position
  auto position = t.transform.translation;
  auto orientation = t.transform.rotation;
  // auto or_msg = t.transform.rotation;
  // Eigen::Quaternionf orientation(or_msg.w, or_msg.x, or_msg.y, or_msg.z);
  
  Eigen::Isometry2f current_pose = Eigen::Isometry2f::Identity();
  // TODO: Set the rotation and translation of the current pose
  current_pose.translate(Eigen::Vector2f(position.x, position.y));
  current_pose.rotate(Eigen::Rotation2Df(2 * atan2(orientation.z, orientation.w)));
  // current_pose.translation() = Eigen::Vector2f(position.x, position.y);
  // current_pose.linear() = orientation.normalized().toRotationMatrix().topLeftCorner<2, 2>();

  _controller->update(current_pose);
  geometry_msgs::msg::Twist twist;
  // twist.linear.x = // TODO: Get the linear velocity from the controller;
  twist.linear.x = _controller->getV();
  // twist.angular.z = // TODO: Get the angular velocity from the controller;
  twist.angular.z = _controller->getW();

  // TODO: Publish the twist
  _twist_pub->publish(twist);
}