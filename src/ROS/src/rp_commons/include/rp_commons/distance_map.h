#pragma once

#include <limits>

#include "grid.h"
#include "grid_map.h"
#include "grid_mapping.h"

struct DMapCell {
  DMapCell* parent = nullptr;
};

/**
 * @brief A grid map with distance information.
 * Distance values are stored as unsigned int values.
 * The class is designed to be used for Localization or Path Planning.
 */
struct DistanceMap : public Grid_<DMapCell>, public GridMapping {
  using Grid_::inside;
  unsigned int _d2_max;

  inline unsigned int d2_max() const { return _d2_max; }
  inline bool inside(Eigen::Vector2f point) const {
    Eigen::Vector2i idx = worldToGrid(point);
    return Grid_::inside(idx.y(), idx.x());
  }

  /**
   * @brief Initializes the grid using a OccupancyGrid message.
   * After gathering all the obstacles, the function computes the distance map.
   *
   * @param map
   * @return * void
   */
  void loadFromOccupancyGrid(
      const nav_msgs::msg::OccupancyGrid& map,
      unsigned int d2_max = std::numeric_limits<unsigned int>::max());

  /**
   * @brief Computes the distance map given a set of obstacles.
   *
   * @param obstacles
   * @param d2_max
   * @return unsigned int
   */
  unsigned int compute(
      const std::vector<std::pair<unsigned int, unsigned int>>& obstacles,
      unsigned int d2_max = std::numeric_limits<unsigned int>::max());

  void clear();

  /**
   * @brief Computes the squared distance between two cells in grid space.
   * The distance is computed as the squared Euclidean distance between the
   * cells' positions.
   *
   * @param c1
   * @param c2
   * @return int
   */
  inline unsigned int squaredDistance(const DMapCell& c1,
                                      const DMapCell& c2) const {
    auto p1 = getCoordinatesFromPointer(&c1);
    auto p2 = getCoordinatesFromPointer(&c2);
    return (p1.first - p2.first) * (p1.first - p2.first) +
           (p1.second - p2.second) * (p1.second - p2.second);
  }

  /**
   * @brief Extracts the distance map from the DistanceMap and stores it in the
   * target grid.
   *
   * @tparam OutputCellType_ Output grid type, expected [float, double, uint]
   * @param target Output grid
   * @param d2_max Maximum distance squared allowed in the distance map
   */
  template <typename OutputCellType_>
  inline void extractDistancesSquared(Grid_<OutputCellType_>& target,
                                      unsigned int d2_max) const {
    target.resize(rows(), cols());
    for (unsigned int i = 0; i < _cells.size(); ++i) {
      const auto& cell = _cells[i];
      unsigned int d2 = d2_max;
      if (cell.parent) {
        d2 = squaredDistance(cell, *cell.parent);
      }
      target._cells[i] = static_cast<OutputCellType_>(d2);
    }
  }

  template <typename OutputCellType_>
  inline void extractDistances(Grid_<OutputCellType_>& target,
                               float d_max) const {
    target.resize(rows(), cols());
    for (unsigned int i = 0; i < _cells.size(); ++i) {
      const auto& cell = _cells[i];
      unsigned int d = d_max;
      if (cell.parent) {
        d = std::min(d_max, sqrtf(squaredDistance(cell, *cell.parent)));
      }
      target._cells[i] = static_cast<OutputCellType_>(d);
    }
  }
};