#pragma once
#include "rp_commons/grid_map.h"
#include "rp_commons/grid_mapping.h"
#include "rp_planner/planner.h"

struct PCell {
  static const unsigned char UNVISITED = 0;
  static const unsigned char MARKED = 1;
  static const unsigned char UNREACHABLE = 2;
  PCell* parent;
  float cost;
  unsigned char state;
};

struct PlanningMap : public Grid_<PCell>, public GridMapping {
  /**
   * @brief Loads the grid map data into the planner.
   *
   * This function initializes the planner's internal representation of the grid
   * map by copying the dimensions, origin, and resolution from the provided
   * GridMap object. It also initializes each cell in the planner's grid with
   * default values.
   *
   * @param map The GridMap object containing the grid data to be loaded.
   */
  void loadFromGridMap(const GridMap& map);

  // Check if a point is inside the map
  inline bool inside(Eigen::Vector2f point) const {
    Eigen::Vector2i idx = worldToGrid(point);
    return Grid_::inside(idx.y(), idx.x());
  }

  bool findShortestPath(Eigen::Vector2f start, Eigen::Vector2f goal,
                        std::vector<Eigen::Vector2f>& path);

  inline bool inside(unsigned int row, unsigned int col) const {
    return Grid_::inside(row, col);
  }
};

/**
 * @brief Dijkstra planner
 * The Dijkstra planner is a derived class of the Planner class that implements
 * the Dijkstra algorithm for path planning.
 */

class PlannerDijkstra : public Planner {
 public:
  std::vector<Eigen::Vector2f> plan() override;

 protected:
  PlanningMap _costmap;
};