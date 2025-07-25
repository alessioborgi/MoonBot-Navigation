#include "rp_commons/distance_map.h"

#include <iostream>
#include <queue>

void DistanceMap::loadFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map,
                                        unsigned int d2_max) {
  _rows = map.info.height;
  _cols = map.info.width;
  _cells.resize(_rows * _cols);
  _origin =
      Eigen::Vector2f(map.info.origin.position.x, map.info.origin.position.y);
  _resolution = map.info.resolution;
  _d2_max = d2_max;

  std::vector<std::pair<unsigned int, unsigned int>> obstacles;

  for (unsigned int r = 0; r < _rows; ++r) {
    for (unsigned int c = 0; c < _cols; ++c) {
      unsigned int i = _cols * (_rows - r - 1) + c;
      unsigned char value = map.data[i];
      Grid_::at(r, c).parent = nullptr;
      if (value == GridMap::OCCUPIED) {
        obstacles.push_back(std::make_pair(r, c));
      }
    }
  }

  compute(obstacles, d2_max);
}

unsigned int DistanceMap::compute(
    const std::vector<std::pair<unsigned int, unsigned int>>& obstacles,
    unsigned int d2_max) {
  _d2_max = d2_max;
  std::queue<DMapCell*> frontier;

  // Initialize the frontier by pushing the obstacles into it
  for (const auto& obs : obstacles) {
    const auto row = obs.first, col = obs.second;
    // Check if the obstacle is inside the grid
    if (!Grid_::inside(row, col)) {
      continue;
    }
    // Initialize the DMapCell by setting the parent to its own cell
    auto& cell = at(row, col);
    at(row, col).parent = &cell;
    frontier.push(&cell);
  }

  unsigned int steps = 0;
  while (!frontier.empty()) {
    ++steps;

    // Fetch one node from the frontier
    auto cell = frontier.front();
    frontier.pop();

    // Fetch the cell's position and its parent
    const auto cell_position = getCoordinatesFromPointer(cell);
    const auto cell_row = cell_position.first;
    const auto cell_col = cell_position.second;
    const auto cell_parent = cell->parent;

    // Visit all neighbors
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        // Skip the current cell
        if (dr == 0 and dc == 0) {
          continue;
        }

        // Compute the neighbor's position
        const auto neighbor_row = cell_row + dr;
        const auto neighbor_col = cell_col + dc;

        // Check if the neighbor is inside the grid
        if (!Grid_::inside(neighbor_row, neighbor_col)) {
          continue;
        }

        // Fetch the neighbor's cell
        auto& neighbor = at(neighbor_row, neighbor_col);

        // cell's parent
        const auto d2_neighbor = squaredDistance(neighbor, *cell_parent);
        if (d2_neighbor > d2_max) {
          continue;
        }
        // If the neighbor has no parent or the distance to the parent is
        // smaller than the current distance, update the parent and push the
        // neighbor to the frontier
        if (!neighbor.parent or
            d2_neighbor < squaredDistance(*neighbor.parent, neighbor)) {
          neighbor.parent = cell_parent;
          frontier.push(&neighbor);
        }
      }
    }
  }
  return steps;
}