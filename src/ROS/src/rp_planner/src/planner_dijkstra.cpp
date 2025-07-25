#include "rp_planner/planner_dijkstra.h"

#include <iostream>
#include <queue>

void PlanningMap::loadFromGridMap(const GridMap& map) {
  _rows = map.rows();
  _cols = map.cols();
  _cells.resize(_rows * _cols);
  _origin = map.origin();
  // _origin = Eigen::Vector2f(_rows * 0.5, _cols * 0.5);
  _resolution = map.resolution();

  for (unsigned int r = 0; r < _rows; ++r) {
    for (unsigned int c = 0; c < _cols; ++c) {
      at(r, c).cost = std::numeric_limits<float>::infinity();
      at(r, c).parent = nullptr;
      at(r, c).state =
          map.at(r, c) == GridMap::FREE ? PCell::UNVISITED : PCell::UNREACHABLE;
    }
  }
}

struct PCellComparator {
  bool operator()(const PCell* lhs, const PCell* rhs) const {
    return lhs->cost > rhs->cost;
  }
};

bool PlanningMap::findShortestPath(Eigen::Vector2f start, Eigen::Vector2f goal,
                                   std::vector<Eigen::Vector2f>& path) {
  const auto start_position = worldToGrid(start);
  const auto goal_position = worldToGrid(goal);

  std::priority_queue<PCell*, std::vector<PCell*>, PCellComparator>
      unvisited_set;

  // Update the start cell
  auto& start_cell = at(start_position.y(), start_position.x());
  start_cell.cost = 0;
  start_cell.state = PCell::MARKED;
  unvisited_set.push(&start_cell);

  // Dijkstra algorithm loop
  while (!unvisited_set.empty()) {
    auto current_cell = unvisited_set.top();
    unvisited_set.pop();

    // Check if the goal has been reached
    if (current_cell == &at(goal_position.y(), goal_position.x())) {
      break;
    }

    // Check the neighbors
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) {
          continue;
        }

        const auto current_cell_pos = getCoordinatesFromPointer(current_cell);
        unsigned int r = current_cell_pos.first + dr;
        unsigned int c = current_cell_pos.second + dc;

        if (inside(r, c)) {
          auto& neighbor = at(r, c);
          if (neighbor.state == PCell::UNREACHABLE) {
            continue;
          }

          float new_cost = current_cell->cost;
          // If traversing the diagonal, then cost increases by sqrt(2)
          if (dr * dc != 0) {
            new_cost += 1.414;
          } else {
            new_cost += 1;
          }

          if (new_cost < neighbor.cost) {
            neighbor.cost = new_cost;
            neighbor.parent = current_cell;
            if (neighbor.state != PCell::MARKED) {
              neighbor.state = PCell::MARKED;
              unvisited_set.push(&neighbor);
            }
          }
        }
      }
    }
  }

  path.clear();
  if (at(goal_position.y(), goal_position.x()).state != PCell::MARKED) {
    return false;
  }

  PCell* current_cell = &at(goal_position.y(), goal_position.x());
  while (current_cell->parent != nullptr) {
    const auto current_cell_pos = getCoordinatesFromPointer(current_cell);
    path.push_back(gridToWorld(
        Eigen::Vector2i(current_cell_pos.second, current_cell_pos.first)));
    current_cell = current_cell->parent;
  }
  std::reverse(path.begin(), path.end());
  return true;
}

std::vector<Eigen::Vector2f> PlannerDijkstra::plan() {
  // Initialize the costmap with the grid map data
  _costmap.loadFromGridMap(_map);

  std::vector<Eigen::Vector2f> computed_path;
  if (isStartValid() && isGoalValid()) {
    _costmap.findShortestPath(_start, _goal, computed_path);
  }

  return computed_path;
}