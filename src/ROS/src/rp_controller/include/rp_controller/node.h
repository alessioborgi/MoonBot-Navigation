#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rp_commons/laser_scan.h"
#include "rp_controller/differential_drive_controller.h"

class DifferentialDriveControllerNode : public rclcpp::Node {
 public:
  DifferentialDriveControllerNode();

  void setObstacleAvoidance() { _controller->setObstacleAvoidance(); }

 private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void update();

  std::mutex _measurements_mutex;

  std::string _laser_ns;
  std::string _base_link_ns;

  std::shared_ptr<DiffDriveController> _controller;
  rclcpp::TimerBase::SharedPtr _timer;

  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _twist_pub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
};