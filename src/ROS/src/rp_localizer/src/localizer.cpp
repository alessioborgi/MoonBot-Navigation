#include "rp_localizer/localizer.h"

#include <iostream>

/**
 * @brief This function initializes the auxiliary structures needed to
 * perform the localization.
 * Once the DistanceMap has been set, the function extracts the metric
 * distance map and the gradients of the distance map.
 *
 */
void Localizer2D::setup() {
  // Compute the distance map in metric space
  _map.extractDistancesSquared(_metric_dmap, _map.d2_max());
  for (auto& cell : _metric_dmap._cells) {
    cell = std::sqrt(cell) * _map.resolution(); // from pixel^2 to meters
  }
  // Compute gradients of the distance map
  computeGradients(_metric_dmap, _dmap_drows, _dmap_dcols);
  // Initialize the pose to the identity
  _pose.setIdentity();
}

const Eigen::Isometry2f& Localizer2D::update(const LaserScan& scan,
                                             unsigned int num_iterations) {
  // Convert the laser scan to points in the local frame
  std::vector<Eigen::Vector2f> measurements = scan.toCartesian();

  // Setup the ICP algorithm
  unsigned int inliers = 0;
  float chi2 = 0.f;

  for (unsigned int i = 0; i < num_iterations; ++i) {
    Eigen::Matrix3f H;
    Eigen::Vector3f b;
    Eigen::Matrix<float, 2, 3> J_icp;
    Eigen::Matrix2f J_gm;
    J_gm << 1.0 / _map.resolution(), 0, 0, -1.0 / _map.resolution();
    Eigen::Matrix<float, 1, 2> J_dmap;
    Eigen::Matrix<float, 1, 3> J;

    J_icp.block<2, 2>(0, 0).setIdentity();
    inliers = 0;
    chi2 = 0.f;
    H.setZero();
    b.setZero();

    for (const auto& m : measurements) {
      Eigen::Vector2f p_world = _pose * m;
      Eigen::Vector2i p_grid = _map.worldToGrid(p_world);
      if (!_map.inside(p_grid)) {
        continue;
      }
      float e = _metric_dmap.at(p_grid);
      float e2 = e * e;
      float lambda = 1.f;
      if (e2 > _kernel_chi2) {
        lambda = _kernel_chi2 / sqrt(e2);
      }

      J_icp.col(2) << -p_world.y(), p_world.x();
      J_dmap.x() = _dmap_dcols.at(p_grid);
      J_dmap.y() = _dmap_drows.at(p_grid);
      J = J_dmap * J_gm * J_icp;

      H += lambda * J.transpose() * J;
      b += lambda * J.transpose() * e;
      chi2 += e2;
      ++inliers;
    }

    H += Eigen::Matrix3f::Identity() * _damping;
    Eigen::Vector3f dx = H.ldlt().solve(-b);
    Eigen::Isometry2f dX;
    dX.translation() << dx.x(), dx.y();
    dX.linear() = Eigen::Rotation2Df(dx.z()).toRotationMatrix();
    _pose = dX * _pose;

    // Debug information
    // std::cerr << "Iteration: " << i << " Inliers: " << inliers
    //           << " Chi2: " << chi2 << " dx: " << dx.transpose() << std::endl;

    // if (dx.norm() < 1e-5) {
    //   break;
    // }
  }
  return _pose;
}

void Localizer2D::setInitialGuess(const Eigen::Isometry2f& initial_guess) {
  _pose = initial_guess;
}