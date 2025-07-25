#pragma once
#include <Eigen/Dense>
#include <limits>
#include <vector>

#include "rp_commons/distance_map.h"
#include "rp_commons/grid.h"
#include "rp_commons/grid_mapping.h"
#include "rp_planner/planner.h"

class PlanningMapV2 : public Planner {
 public:
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
  void loadFromGridMap(const GridMap& map, const DistanceMap& dmap);

  std::vector<Eigen::Vector2f> plan() override;

  void computePolicy();
  struct DijkstraCell {
    DijkstraCell(DijkstraCell* p = 0,
                 float c = std::numeric_limits<float>::max())
        : parent(p), cost(c) {}
    DijkstraCell* parent;
    float cost;
  };

  using DijkstraGrid = Grid_<DijkstraCell>;

 protected:
  inline float traversalCost(const Eigen::Vector2i& from,
                             const Eigen::Vector2i& to) const {
    return sqrt((from - to).squaredNorm()) * _obstacle_costs.at(to.x(), to.y());
  }

  Grid_<float> _costmap;
  Grid_<float> _obstacle_costs;
  Grid_<float> _policy;
  DijkstraGrid _d_grid;
  float _max_traversal_cost = 100;
  float _max_distance_range = 5.0;
  bool _policy_ok;
};