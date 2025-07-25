#include "rp_planner/planner_dijkstra_v2.hpp"

#include "rp_commons/distance_map.h"

void PlanningMapV2::loadFromGridMap(const GridMap& map, const DistanceMap& dmap) {
  _map = map;
  const float robot_radius = this->_robot_radius;
  const float max_range = this->_max_range;
  _max_distance_range = max_range;
  float max_range_in_pixels = max_range / map.resolution();

  Grid_<float> distances;
  dmap.extractDistancesSquared(distances, max_range_in_pixels);

  _obstacle_costs.resize(map.rows(), map.cols());
  float d_slope = _max_traversal_cost / (max_range - robot_radius);
  for (unsigned int r = 0; r < map.rows(); ++r) {
    for (unsigned int c = 0; c < map.cols(); ++c) {
      const float distance_in_pixels = distances.at(r, c);
      const float distance_in_meters = distance_in_pixels * map.resolution();
      const float delta_cost = distance_in_meters - robot_radius;
      float cost = _max_traversal_cost;
      if (delta_cost > 0) {
        cost = std::max(0.f, _max_traversal_cost - delta_cost * d_slope) +
               map.resolution();
      }
      _obstacle_costs.at(r, c) = cost;
    }
  }
  //   mapping.resize(obstacle_costs.rows, obstacle_costs.cols, resolution);
  _policy_ok = false;
}

struct DijkstraPQCompare {
  const PlanningMapV2& planner;
  DijkstraPQCompare(const PlanningMapV2& p) : planner(p) {}
  inline bool operator()(const PlanningMapV2::DijkstraCell* a,
                         const PlanningMapV2::DijkstraCell* b) const {
    return a->cost > b->cost;
  }
};

using DijkstraFrontier =
    std::priority_queue<PlanningMapV2::DijkstraCell*,
                        std::vector<PlanningMapV2::DijkstraCell*>,
                        DijkstraPQCompare>;

void PlanningMapV2::computePolicy() {
  _d_grid.resize(_map.rows(), _map.cols());
  std::fill(_d_grid._cells.begin(), _d_grid._cells.end(),
            PlanningMapV2::DijkstraCell());
  Eigen::Vector2i goal_i = _map.worldToGrid(_goal);
  std::cerr << goal_i.transpose() << " " << _goal.transpose() << std::endl;
  if (!_obstacle_costs.inside(goal_i)) {
    std::cerr << "Goal is outside the map" << std::endl;
    _policy_ok = false;
    return;
  }
  DijkstraFrontier frontier(*this);
  auto& goal_cell = _d_grid.at(goal_i.y(), goal_i.x());
  goal_cell.parent = &goal_cell;
  goal_cell.cost = 0;
  frontier.push(&goal_cell);
  int num_expansions = 0;
  bool step_mode = true;
  while (!frontier.empty()) {
    auto current = frontier.top();
    ++num_expansions;
    frontier.pop();
    const auto cpose = _d_grid.getCoordinatesFromPointer(current);
    Eigen::Vector2i current_pos = Eigen::Vector2i(cpose.first, cpose.second);
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) continue;
        Eigen::Vector2i d_pos(dr, dc);
        Eigen::Vector2i next_pos = current_pos + d_pos;
        if (!_obstacle_costs.inside(next_pos.x(), next_pos.y())) {
          continue;
        }
        float traversal_cost = traversalCost(current_pos, next_pos);
        if (traversal_cost >= _max_traversal_cost) {
          continue;
        }
        float expected_cost = current->cost + traversal_cost;
        auto& next = _d_grid.at(next_pos.x(), next_pos.y());
        if (expected_cost < next.cost) {
          next.parent = current;
          next.cost = expected_cost;
          frontier.push(&next);
        }
      }
    }
  }
  _policy.resize(_map.rows(), _map.cols());
  std::fill(_policy._cells.begin(), _policy._cells.end(), -1);
  for (size_t i = 0; i < _d_grid._cells.size(); ++i) {
    const auto& cell = _d_grid._cells[i];
    if (!cell.parent) continue;
    _policy._cells[i] = _d_grid._cells[i].cost;
  }
  _policy_ok = true;
}

std::vector<Eigen::Vector2f> PlanningMapV2::plan() {
  float approach_cost = _map.resolution();
  std::vector<Eigen::Vector2f> path;
  if (!_policy_ok) {
    return path;
  }
  const auto start_i = _map.worldToGrid(_start);
  const auto goal_i = _map.worldToGrid(_goal);
  auto current = &_d_grid.at(start_i.y(), start_i.x());
  if (!current->parent) {
    std::cerr << "No parent" << std::endl;
    return path;
  }
  std::cerr << "start_i=" << start_i.x() << " " << start_i.y() << std::endl;
  std::cerr << "goal_i=" << goal_i.x() << " " << goal_i.y() << std::endl;
  std::cerr << "cost=" << current->cost << std::endl;
  while (current->parent != current && current->cost > approach_cost) {
    const auto cpose = _d_grid.getCoordinatesFromPointer(current);
    std::cerr << "current=" << cpose.first << " " << cpose.second << std::endl;
    path.push_back(
        _map.gridToWorld(Eigen::Vector2i(cpose.second, cpose.first)));
    current = current->parent;
  }
  return path;
}