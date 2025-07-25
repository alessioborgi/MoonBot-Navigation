#include "rp_simulator/unicycle.h"

/** @brief Constructs a new UnicyclePlatform object.
 * @param w The world in which the platform exists.
 * @param ns The namespace of the platform.
 * @param node The ROS node to use for timers and publishers.
 * @param iso The pose of the platform in the parent frame.
 * @param publish_tf Whether to publish the transform of the platform.
 */
UnicyclePlatform::UnicyclePlatform(World& w, const std::string& ns,
                                   rclcpp::Node::SharedPtr node,
                                   const Eigen::Isometry2f& iso,
                                   bool publish_tf)
    : WorldItem(w, iso, ns, node),
      _tf_broadcaster(node),
      _publish_tf(publish_tf) {
  // Subscribe to the cmd_vel topic
  _cmd_vel_sub = _node->create_subscription<geometry_msgs::msg::Twist>(
      _namespace + "/cmd_vel", 10,
      std::bind(&UnicyclePlatform::velocityCallback, this,
                std::placeholders::_1));

  // Publisher for groundtruth pose
  _groundtruth_pub = _node->create_publisher<geometry_msgs::msg::PoseStamped>(
      _namespace + "/groundtruth", 10);
}

/** @brief Callback function to handle velocity commands.
 * @param msg The velocity command message.
 */
void UnicyclePlatform::velocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  // Update the robot's velocity
  _tv = msg->linear.x;
  _rv = msg->angular.z;
}

/** @brief Publishes the current pose of the platform.
 * @param time_now The current time.
 */
void UnicyclePlatform::publishPose(rclcpp::Time time_now) {
  // Publish pose_in_parent using _groundtruth_pub
  geometry_msgs::msg::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header.stamp = time_now;
  pose_stamped_msg.header.frame_id = "map";
  pose_stamped_msg.pose.position.x = _pose_in_parent.translation().x();
  pose_stamped_msg.pose.position.y = _pose_in_parent.translation().y();
  pose_stamped_msg.pose.orientation.z = sin(
      atan2(_pose_in_parent.linear()(1, 0), _pose_in_parent.linear()(0, 0)) /
      2);
  pose_stamped_msg.pose.orientation.w = cos(
      atan2(_pose_in_parent.linear()(1, 0), _pose_in_parent.linear()(0, 0)) /
      2);
  _groundtruth_pub->publish(pose_stamped_msg);

  // Publish the transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = time_now;
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = _namespace;
  transform_stamped.transform.translation.x = pose_stamped_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_stamped_msg.pose.position.y;

  transform_stamped.transform.rotation.z = pose_stamped_msg.pose.orientation.z;
  transform_stamped.transform.rotation.w = pose_stamped_msg.pose.orientation.w;
  _tf_broadcaster.sendTransform(transform_stamped);
}

/** @brief Updates the platform's state.
 * @param dt The time delta since the last update.
 * @param time_now The current time.
 */
void UnicyclePlatform::tick(float dt, rclcpp::Time time_now) {
  // Compute the motion based on the current velocities
  Eigen::Isometry2f motion = Eigen::Isometry2f::Identity();
  motion.translation() << _tv * dt, 0;
  motion.linear() = Eigen::Rotation2Df(_rv * dt).matrix();

  // Attempt to move the platform
  if (!move(motion)) {
    std::cerr << "UniciclePlatform| moving fails" << std::endl;
    // If the move fails, stop the platform
    _tv = 0;
    _rv = 0;
  }

  // Call the base class tick method
  WorldItem::tick(dt, time_now);

  // Publish the pose if required
  if (_publish_tf) publishPose(time_now);
}