#pragma once
#include <Eigen/Dense>
#include <vector>

#include "rp_commons/grid_map.h"

/**
 * @brief Abstract interface for planning algorithms
 * The planner class is an abstract class that provides an interface for
 * planning algorithms. It contains a pure virtual function plan() that
 * should be implemented by the derived classes.
 * The planner class also contains a GridMap object that represents the
 * environment, and two Eigen::Vector2f objects that represent the start and
 * goal positions.
 *
 */
class Planner {
 public:
  // Pure virtual function for planning
  virtual std::vector<Eigen::Vector2f> plan() = 0;

  GridMap& getMap() { return _map; }
  const GridMap& getMap() const { return _map; }
  const Eigen::Vector2f& getStart() const { return _start; }
  const Eigen::Vector2f& getGoal() const { return _goal; }

  void setStart(const Eigen::Vector2f& start) { _start = start; }
  void setGoal(const Eigen::Vector2f& goal) { _goal = goal; }

  bool isStartValid() const { return _map.inside(_start); }
  bool isGoalValid() const { return _map.inside(_goal); }

 protected:
  GridMap _map;
  Eigen::Vector2f _start;
  Eigen::Vector2f _goal;
  float _robot_radius = 20.0;
  float _max_range = 400.0;
};