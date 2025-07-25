#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rp_localizer/localizer.h"

class Localizer2DNode : public rclcpp::Node {
 public:
  Localizer2DNode();

 protected:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  // Must be called every time a new laser offset is received
  // (the offset between laser and base_link changes in case of an articulated
  // robot)
  void laserOffsetCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void poseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      _laser_offset_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      _pose_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
  tf2_ros::TransformBroadcaster _tf_broadcaster;

  Localizer2D _localizer;
  std::string _laser_ns;
  std::string _base_link_ns;
  Eigen::Isometry2f _laser_in_base_link;
  std::mutex _laser_in_base_link_mutex;  // Mutex to protect _laser_in_base_link
  bool _pose_initialized = false;
  bool _map_initialized = false;
};