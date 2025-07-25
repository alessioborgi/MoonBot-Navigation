#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>

#include "rp_commons/grid_map.h"

class WorldItem;
using WorldItemSet = std::set<WorldItem *>;

/**
 * @brief The WorldItem class represents an item in the simulated world.
 *
 * This class represents an item in the simulated world. It can be a robot, a
 * sensor, or any other object that can be placed in the world. The WorldItem
 * class provides functionality for moving the item, checking for collisions,
 * and drawing the item on a canvas.
 */
class WorldItem {
 protected:
  /**
   * @brief Constructs a new WorldItem object.
   * @param g The grid map representing the world.
   * @param p The parent WorldItem.
   * @param iso The pose of the WorldItem in the parent frame.
   * @param ns The namespace of the WorldItem.
   * @param node The ROS node to use for timers and publishers.
   */
  WorldItem(const GridMap *g, WorldItem *p,
            const Eigen::Isometry2f &iso = Eigen::Isometry2f::Identity(),
            const std::string &ns = "", rclcpp::Node::SharedPtr node = nullptr);

 public:
  /**
   * @brief Constructs a new WorldItem object.
   * @param g The grid map representing the world.
   * @param ns The namespace of the WorldItem.
   * @param node The ROS node to use for timers and publishers.
   */
  WorldItem(const GridMap &g, const std::string &ns = "",
            rclcpp::Node::SharedPtr node = nullptr)
      : WorldItem(&g, nullptr, Eigen::Isometry2f::Identity(), ns, node) {}

  /** @brief Constructs a new WorldItem object.
   * @param p The parent WorldItem.
   * @param iso The pose of the WorldItem in the parent frame.
   * @param ns The namespace of the WorldItem.
   * @param node The ROS node to use for timers and publishers.
   */
  WorldItem(WorldItem &p,
            const Eigen::Isometry2f &iso = Eigen::Isometry2f::Identity(),
            const std::string &ns = "", rclcpp::Node::SharedPtr node = nullptr)
      : WorldItem(p._grid_map, &p, iso, ns, node) {}

  ~WorldItem();

  /**
   * @brief Get the global pose of the WorldItem.
   * @return The global pose of the WorldItem.
   */
  Eigen::Isometry2f globalPose() const;

  /**
   * @brief Check if the other is an ancestor of the WorldItem.
   * @param other The WorldItem to check if it is an ancestor.
   * @return True if the other is an ancestor of the WorldItem.
   */
  bool isAncestor(const WorldItem &other) const;

  /**
   * @brief Check if the WorldItem is in collision with the map or another item.
   * @return True if the WorldItem is in collision.
   */
  bool checkCollision() const;

  /**
   * @brief Check if the WorldItem is in collision with other WorldItems.
   * @return True if the WorldItem is in collision.
   */
  bool checkCollision(const WorldItem &other) const;

  /**
   * @brief Move the WorldItem by the given transformation.
   * @param iso The transformation to apply.
   * @return True if the move was successful.
   */
  bool move(const Eigen::Isometry2f &iso);

  /**
   * @brief Tick the WorldItem and all its children.
   * @param time_interval The time interval to tick.
   * @param time_now The current time.
   */
  virtual void tick(float time_interval, rclcpp::Time time_now);

  /**
   * @brief Draw the WorldItem on the canvas.
   * @param canvas The canvas to draw on.
   */
  virtual void draw(Canvas &canvas) const;

  const GridMap *_grid_map = 0;
  WorldItem *_parent;
  Eigen::Isometry2f _pose_in_parent;
  float _radius = 1;
  WorldItemSet _children;
  std::string _namespace;
  rclcpp::Node::SharedPtr _node;
};