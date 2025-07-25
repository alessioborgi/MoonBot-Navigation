#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rp_commons/grid_map.h"

/**
 * @brief A node that publishes an occupancy grid map.
 *
 * This node loads a grid map from an image file and publishes it as an
 * OccupancyGrid message.
 */
class MapServerNode : public rclcpp::Node {
 public:
  MapServerNode();

 protected:
  /**
   * @brief Publishes the occupancy grid map.
   *
   * This function is called periodically by the timer to publish the current
   * state of the occupancy grid map.
   */
  void publishMap();
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      _occupancy_grid_pub;
  rclcpp::TimerBase::SharedPtr _timer_map;  // Timer for publishing the map

  GridMap _grid_map;
  const float _map_interval = 1.0;  // Interval for publishing the map
};
