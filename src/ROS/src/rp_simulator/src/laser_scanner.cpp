#include "rp_simulator/laser_scanner.h"
/**
 * @brief Constructs a new LaserScanner object.
 * @param par The parent WorldItem.
 * @param ns The namespace of the LaserScanner.
 * @param node The ROS node to use for timers and publishers.
 * @param pos The pose of the LaserScanner in the parent frame.
 * @param f The frequency of the LaserScanner.
 */
LaserScanner::LaserScanner(WorldItem& par, const std::string& ns,
                           rclcpp::Node::SharedPtr node,
                           const Eigen::Isometry2f& pos, float f)
    : WorldItem(par, pos),
      _node(node),
      _namespace(ns),
      _frequency(f),
      _period(1. / f),
      _tf_broadcaster(node) {
  // Initialize publishers for LaserScan and PoseStamped messages
  _laser_scan_pub = _node->create_publisher<sensor_msgs::msg::LaserScan>(
      _namespace + "/scan", 10);
  _laser_in_base_link_pub =
      _node->create_publisher<geometry_msgs::msg::PoseStamped>(
          _namespace + "/offset", 10);
}

/**
 * @brief Performs a laser scan and updates the scan data.
 */
void LaserScanner::getScan() {
  Eigen::Isometry2f gp = globalPose();
  Eigen::Isometry2f rotation = gp;
  rotation.translation().setZero();
  float angle_increment =
      (_scan._angle_max - _scan._angle_min) / _scan._ranges.size();

  for (size_t i = 0; i < _scan._ranges.size(); ++i) {
    float beam_angle = _scan._angle_min + angle_increment * i;
    Eigen::Vector2f d(cos(beam_angle), sin(beam_angle));
    d = rotation * d;
    _scan._ranges[i] =
        _grid_map->scanRay(gp.translation(), d, _scan._range_max);
  }
}

/**
 * @brief Publishes the laser scan data and the transform.
 * @param time_now The current time.
 */
void LaserScanner::publishLaserScan(rclcpp::Time time_now) {
  // Create and populate LaserScan message
  sensor_msgs::msg::LaserScan msg;
  msg.header.frame_id = _namespace;
  msg.header.stamp = time_now;
  msg.angle_min = _scan._angle_min;
  msg.angle_max = _scan._angle_max;
  msg.angle_increment = _scan._angle_increment;
  msg.range_min = _scan._range_min;
  msg.range_max = _scan._range_max;
  msg.ranges = _scan._ranges;
  msg.time_increment = 0;
  msg.scan_time = 0;

  // Publish LaserScan message
  _laser_scan_pub->publish(msg);

  // Calculate pose in base link frame
  Eigen::Isometry2f pose_in_base_link = Eigen::Isometry2f::Identity();
  WorldItem* base_link = this;
  while (base_link->_parent->_parent) {
    pose_in_base_link = base_link->_pose_in_parent * pose_in_base_link;
    base_link = base_link->_parent;
  }

  // Publish pose_in_base_link using _laser_in_base_link_pub
  geometry_msgs::msg::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header.stamp = time_now;
  pose_stamped_msg.header.frame_id = base_link->_namespace;
  pose_stamped_msg.pose.position.x = pose_in_base_link.translation().x();
  pose_stamped_msg.pose.position.y = pose_in_base_link.translation().y();
  pose_stamped_msg.pose.orientation.z =
      sin(atan2(pose_in_base_link.linear()(1, 0),
                pose_in_base_link.linear()(0, 0)) /
          2);
  pose_stamped_msg.pose.orientation.w =
      cos(atan2(pose_in_base_link.linear()(1, 0),
                pose_in_base_link.linear()(0, 0)) /
          2);
  _laser_in_base_link_pub->publish(pose_stamped_msg);

  // Publish the transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = pose_stamped_msg.header.stamp;
  transform_stamped.header.frame_id = base_link->_namespace;
  transform_stamped.child_frame_id = _namespace;
  transform_stamped.transform.translation.x = pose_stamped_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_stamped_msg.pose.position.y;

  transform_stamped.transform.rotation.z = pose_stamped_msg.pose.orientation.z;
  transform_stamped.transform.rotation.w = pose_stamped_msg.pose.orientation.w;
  _tf_broadcaster.sendTransform(transform_stamped);
}

/**
 * @brief Updates the scanner state and publishes data if necessary.
 * @param dt The time delta since the last update.
 * @param time_now The current time.
 */
void LaserScanner::tick(float dt, rclcpp::Time time_now) {
  _elapsed_time += dt;
  _new_scan = false;
  if (_elapsed_time < _period) return;

  getScan();
  _elapsed_time = 0;
  _new_scan = true;

  publishLaserScan(time_now);
}

/**
 * @brief Draws the laser scan on the provided canvas.
 * @param canvas The canvas to draw on.
 */
void LaserScanner::draw(Canvas& canvas) const {
  _scan.draw(canvas, *_grid_map, globalPose());
}