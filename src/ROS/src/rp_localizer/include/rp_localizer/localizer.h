#pragma once

#include <Eigen/Dense>

#include "rp_commons/distance_map.h"
#include "rp_commons/distance_map_utils.h"
#include "rp_commons/grid.h"
#include "rp_commons/laser_scan.h"

/**
 * @brief Localizer2D provides a 2D localization algorithm based on
 * Iterative Closest Point (ICP) using DistanceMap for fast
 * nearest-neighbor lookups.
 */
class Localizer2D {
 public:
  /**
   * @brief This function initializes the auxiliary structures needed to
   * perform the localization.
   * Once the DistanceMap has been set, the function extracts the metric
   * distance map and the gradients of the distance map.
   *
   */
  void setup();
  const Eigen::Isometry2f& update(const LaserScan& scan,
                                  unsigned int num_iterations = 100);
  void setInitialGuess(const Eigen::Isometry2f& initial_guess);

  inline DistanceMap& getMap() { return _map; }
  inline const DistanceMap& getMap() const { return _map; }

  inline const Eigen::Isometry2f& pose() const { return _pose; }

 protected:
  Eigen::Isometry2f _pose;
  DistanceMap _map;
  Grid_<float> _metric_dmap, _dmap_drows, _dmap_dcols;

  // ICP parameters
  float _damping = 1.f;
  float _kernel_chi2 = 1.f;
  unsigned int _min_inliers = 10;
};