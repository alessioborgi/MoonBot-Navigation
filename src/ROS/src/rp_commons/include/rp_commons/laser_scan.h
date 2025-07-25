#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>

#include "grid_map.h"

/**
 * @brief LaserScan represent a 2D laser scan.
 * The scan is represented as a set of ranges, with the corresponding
 * angle for each range (Polar representation).
 * The scan is expressed in the sensor frame.
 * The class provides convinient methods to pass from and to ROS messages.
 */
struct LaserScan {
  float _range_min, _range_max, _angle_min, _angle_max, _angle_increment;
  int _ranges_num;
  std::vector<float> _ranges;

  LaserScan(float range_min = 0.1, float range_max = 10,
            float angle_min = -M_PI / 2, float angle_max = M_PI / 2,
            int ranges_num = 180);

  /**
   * @brief Convert the laser scan to a vector of points in the Cartesian
   * space.
   *
   * @return std::vector<Eigen::Vector2f>
   */
  std::vector<Eigen::Vector2f> toCartesian() const;

  /**
   * @brief Convert a sensor_msgs::msg::LaserScan message to a LaserScan object.
   *
   * @param msg
   */
  inline void fromROSMessage(const sensor_msgs::msg::LaserScan& msg) {
    _range_min = msg.range_min;
    _range_max = msg.range_max;
    _angle_min = msg.angle_min;
    _angle_max = msg.angle_max;
    _angle_increment = msg.angle_increment;
    _ranges_num = msg.ranges.size();
    _ranges = msg.ranges;
  }

  /**
   * @brief Convert a LaserScan object to a sensor_msgs::msg::LaserScan message.
   * Header information is not included and should be set by the user
   * separately.
   *
   * @param msg
   */
  inline void toROSMessage(sensor_msgs::msg::LaserScan& msg) const {
    msg.range_min = _range_min;
    msg.range_max = _range_max;
    msg.angle_min = _angle_min;
    msg.angle_max = _angle_max;
    msg.angle_increment = _angle_increment;
    msg.ranges = _ranges;
  }

  void draw(Canvas& canevasso, const GridMap& grid_map,
            const Eigen::Isometry2f& pose) const;
};