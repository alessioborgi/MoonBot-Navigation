#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>

#include "rp_commons/grid_map.h"
#include "world_item.h"

/**
 * @brief The World class represents a simulated world.
 *
 * This class inherits from WorldItem and manages the simulation of a world,
 * including ticking the world state and publishing the occupancy grid map.
 */
class World : public WorldItem {
 public:
  /**
   * @brief Constructs a new World object.
   *
   * @param gmap The grid map representing the world.
   * @param node The ROS node to use for timers and publishers.
   * @param tick_interval The interval at which the world state is updated.
   */
  World(const GridMap &gmap, rclcpp::Node::SharedPtr node, float tick_interval)
      : WorldItem(gmap, "", node), _tick_interval(tick_interval) {
    // Create a timer inside World for ticking the world
    _timer_tick =
        _node->create_wall_timer(std::chrono::duration<float>(_tick_interval),
                                 std::bind(&World::clock, this));
  }

  /**
   * @brief Ticks the world and all its children.
   *
   * This function is called periodically by the timer to update the world
   * state.
   */
  void clock() {
    WorldItem::tick(_tick_interval,
                    _node->now());  // Call the base class tick method
  }

  /**
   * @brief Draws the world onto the given canvas.
   *
   * This function draws the grid map on the provided canvas. It also
   * recursively draws all child elements of the world.
   *
   * @param canvas The canvas on which to draw the world.
   */
  void draw(Canvas &canvas) const {
    _grid_map->draw(canvas);

    for (auto _child : _children) _child->draw(canvas);
  }

 protected:
  rclcpp::TimerBase::SharedPtr _timer_tick;  // Timer for ticking the world
  float _tick_interval;                      // Time interval for each tick
};