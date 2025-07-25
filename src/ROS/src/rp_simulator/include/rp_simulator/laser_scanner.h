#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rp_commons/laser_scan.h"
#include "world_item.h"

class LaserScanner : public WorldItem {
 public:
  /**
   * @brief Constructs a new LaserScanner object.
   * @param par The parent WorldItem.
   * @param ns The namespace of the LaserScanner.
   * @param node The ROS node to use for timers and publishers.
   * @param pos The pose of the LaserScanner in the parent frame.
   * @param f The frequency of the LaserScanner.
   */
  LaserScanner(WorldItem& par, const std::string& ns,
               rclcpp::Node::SharedPtr node, const Eigen::Isometry2f& pos,
               float f);

  /**
   * @brief Checks if a new scan is available.
   * @return True if a new scan is available, false otherwise.
   */
  inline bool newScan() const { return _new_scan; }

  /**
   * @brief Performs a laser scan and updates the scan data.
   */
  void getScan();
  /**
   * @brief Publishes the laser scan data and the transform.
   * @param time_now The current time.
   */
  void publishLaserScan(rclcpp::Time time_now);

  /**
   * @brief Updates the scanner state and publishes data if necessary.
   * @param dt The time delta since the last update.
   * @param time_now The current time.
   */
  void tick(float dt, rclcpp::Time time_now);
  /**
   * @brief Draws the laser scan on the provided canvas.
   * @param canvas The canvas to draw on.
   */
  void draw(Canvas& canvas) const;

 protected:
  LaserScan _scan{0.1, 100, -M_PI / 2, M_PI / 2, 180};  // Laser scan parameters
  float _elapsed_time = 0;        // Elapsed time since last scan
  bool _new_scan = false;         // Flag indicating if a new scan is available
  rclcpp::Node::SharedPtr _node;  // ROS node
  std::string _namespace;         // Namespace for the scanner
  float _frequency;               // Frequency of the scanner
  float _period;                  // Period of the scanner
  tf2_ros::TransformBroadcaster _tf_broadcaster;  // Transform broadcaster
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      _laser_scan_pub;  // Laser scan publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      _laser_in_base_link_pub;  // PoseStamped publisher
};
