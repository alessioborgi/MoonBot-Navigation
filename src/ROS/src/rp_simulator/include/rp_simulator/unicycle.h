#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>

#include "rp_commons/grid_map.h"
#include "world.h"
#include "world_item.h"

class UnicyclePlatform : public WorldItem {
 public:
  /** @brief Constructs a new UnicyclePlatform object.
   * @param w The world in which the platform exists.
   * @param ns The namespace of the platform.
   * @param node The ROS node to use for timers and publishers.
   * @param iso The pose of the platform in the parent frame.
   * @param publish_tf Whether to publish the transform of the platform.
   */
  UnicyclePlatform(World& w, const std::string& ns,
                   rclcpp::Node::SharedPtr node, const Eigen::Isometry2f& iso,
                   bool publish_tf);
  /** @brief Callback function to handle velocity commands.
   * @param msg The velocity command message.
   */
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /** @brief Publishes the current pose of the platform.
   * @param time_now The current time.
   */
  void publishPose(rclcpp::Time time_now);
  /** @brief Updates the platform's state.
   * @param dt The time delta since the last update.
   * @param time_now The current time.
   */
  void tick(float dt, rclcpp::Time time_now);

 protected:
  float _tv = 0;  // Translational velocity
  float _rv = 0;  // Rotational velocity
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      _cmd_vel_sub;  // Subscriber for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      _groundtruth_pub;  // Publisher for groundtruth pose
  tf2_ros::TransformBroadcaster _tf_broadcaster;  // Transform broadcaster
  bool _publish_tf =
      false;  // Flag to indicate whether to publish the transform
};
