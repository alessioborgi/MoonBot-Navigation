#include "rp_simulator/world_item.h"

#include <iostream>

WorldItem::~WorldItem() {
  if (_parent) _parent->_children.erase(this);
}

WorldItem::WorldItem(const GridMap* g, WorldItem* p,
                     const Eigen::Isometry2f& iso, const std::string& ns,
                     rclcpp::Node::SharedPtr node)
    : _grid_map(g),
      _parent(p),
      _pose_in_parent(iso),
      _namespace(ns),
      _node(node) {
  if (!p) return;             // If no parent, return
  p->_children.insert(this);  // Add this item to the parent's children set
}

bool WorldItem::isAncestor(const WorldItem& other) const {
  const WorldItem* current = this;
  while (current) {
    if (current == &other)
      return true;  // If the current item is the same as other, other is
                    // ancestor of this
    current = current->_parent;  // Move up the hierarchy to the parent
  }
  return false;  // If we reach the top without finding other, it is not an
                 // ancestor
}

Eigen::Isometry2f WorldItem::globalPose() const {
  if (!_parent) return _pose_in_parent;
  return _parent->globalPose() * _pose_in_parent;
}

bool WorldItem::checkCollision() const {
  // Get the global pose of the current item
  Eigen::Isometry2f pose = globalPose();

  // Calculate the radius in pixels
  int radius_px = _radius / _grid_map->_resolution;
  int r2 = radius_px * radius_px;

  // Convert the world coordinates to grid coordinates
  Eigen::Vector2f translation = pose.translation();
  Eigen::Vector2i origin_px = _grid_map->worldToGrid(translation);
  int r0 = origin_px.y();
  int c0 = origin_px.x();

  // Check for collision with the map
  for (int r = -radius_px; r <= radius_px; ++r) {
    for (int c = -radius_px; c <= radius_px; ++c) {
      // Skip points outside the circle
      if (r * r + c * c > r2) continue;

      // Check if the point is inside the grid map
      if (!_grid_map->inside(static_cast<int>(r + r0),
                             static_cast<int>(c + c0))) {
        std::cerr << "Outside the map pix | r=" << r + r0 << " c=" << c + c0
                  << std::endl;
        return true;
      }

      // Check if the point is occupied
      if (_grid_map->isOccupied(r + r0, c + c0)) {
        std::cerr << "Collision with occupied pix | r=" << r + r0
                  << " c=" << c + c0 << std::endl;
        return true;
      }
    }
  }

  // Check for collision with child items
  for (auto child : _children)
    if (child->checkCollision()) return true;

  // Check for collision with sibling items
  if (_parent && !(_parent->_parent)) {
    for (const auto* sibling : _parent->_children) {
      if (sibling == this) continue;
      if (checkCollision(*sibling)) return true;
      if (sibling->checkCollision(*this)) return true;
    }
  }

  // No collision detected
  return false;
}

bool WorldItem::checkCollision(const WorldItem& other) const {
  // If other is an ancestor of this item, no collision is possible
  if (isAncestor(other)) return false;

  // If this item is an ancestor of other, no collision is possible
  if (other.isAncestor(*this)) return false;

  // Calculate the global poses of both items
  Eigen::Isometry2f my_pose = globalPose();
  Eigen::Isometry2f other_pose = other.globalPose();

  // Calculate the distance between the two items
  Eigen::Vector2f delta = my_pose.translation() - other_pose.translation();
  float distance = delta.norm();

  // Check if the distance is less than the sum of their radii (collision)
  if (distance < (_radius + other._radius)) return true;

  // Check for collision with child items
  for (auto child : _children)
    if (child->checkCollision(other)) return true;

  // No collision detected
  return false;
}

bool WorldItem::move(const Eigen::Isometry2f& iso) {
  // Save the current pose in parent to restore it in case of collision
  Eigen::Isometry2f restored_pose_in_parent = _pose_in_parent;

  // Apply the transformation to the current pose
  _pose_in_parent = _pose_in_parent * iso;

  // Check for collision after the move
  if (checkCollision()) {
    // If collision detected, restore the original pose
    _pose_in_parent = restored_pose_in_parent;
    return false;  // Move failed due to collision
  }

  return true;  // Move successful
}

void WorldItem::tick(float dt, rclcpp::Time time_now) {
  for (auto child : _children) child->tick(dt, time_now);
}

void WorldItem::draw(Canvas& canvas) const {
  // Convert the global position of the item to grid coordinates
  Eigen::Vector2i center = _grid_map->worldToGrid(globalPose().translation());

  // Calculate the radius in pixels
  int radius_px = _radius / _grid_map->resolution();

  // Draw the item as a circle on the canvas
  drawCircle(canvas, center.cast<int>(), radius_px, 0);

  // Calculate the position of the x-axis in the item's local frame
  Eigen::Vector2f x_in_item = {_radius, 0};

  // Transform the x-axis position to the world frame
  Eigen::Vector2f x_in_world = globalPose() * x_in_item;

  // Convert the x-axis position to grid coordinates
  Eigen::Vector2i x_in_grid = _grid_map->worldToGrid(x_in_world);

  // Draw a line representing the x-axis of the item
  drawLine(canvas, center.cast<int>(), x_in_grid.cast<int>(), 0);

  // Recursively draw all child items
  for (auto child : _children) child->draw(canvas);
}