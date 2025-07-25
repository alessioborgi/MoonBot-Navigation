#pragma once
#include <Eigen/Dense>

/**
 * @brief GridMapping represents a mapping between grid indices and world
 * coordinates.
 */
struct GridMapping {
  Eigen::Vector2f _origin; // [m]
  double _resolution;  // [m / pixels]

  GridMapping() {}

  /**
   * @brief Converts a pixel coordinate to a world coordinate.
   *
   * @param pixel Pixel coordinate.
   * @return Eigen::Vector2f World coordinate.
   */
  inline Eigen::Vector2f gridToWorld(const Eigen::Vector2f& pixel) const {
    Eigen::Vector2f dest;
    dest.x() = (pixel.x() * _resolution) + _origin.x();
    dest.y() = -(1 + pixel.y()) * _resolution;
    return dest;
  }

  /**
   * @brief Converts a pixel coordinate to a world coordinate.
   *
   * @param pixel Pixel coordinate.
   * @return Eigen::Vector2f World coordinate.
   */
  inline Eigen::Vector2f gridToWorld(const Eigen::Vector2i& pixel) const {
    return gridToWorld((Eigen::Vector2f)pixel.cast<float>());
  }

  /**
   * @brief Converts a world coordinate to a pixel coordinate.
   *
   * @param point World coordinate.
   * @return Eigen::Vector2i Pixel coordinate.
   */
  inline Eigen::Vector2i worldToGrid(const Eigen::Vector2f& point) const {
    Eigen::Vector2i dest;
    dest.x() = (point.x() - _origin.x()) / _resolution;
    dest.y() = (-point.y()) / _resolution - 1;
    return dest;
  }

  inline const Eigen::Vector2f& origin() const { return _origin; }
  inline double resolution() const { return _resolution; }
};