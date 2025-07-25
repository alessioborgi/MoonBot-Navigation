#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "draw_helpers.h"
#include "grid.h"
#include "grid_mapping.h"

/**
 * @brief A grid map with occupancy information.
 * Occupancy values are stored as unsigned char (uint8_t) values.
 * The class is designed to be used for Localization and Path Planning
 *
 */
struct GridMap : public Grid_<unsigned char>, public GridMapping {
  static const unsigned char FREE = 0;
  static const unsigned char OCCUPIED = 100;
  static const unsigned char UNKNOWN = 255;

  using Grid_::at;
  using Grid_::inside;

  GridMap() {}

  /**
   * @brief Constructs a grid map with the given number of rows and columns.
   *
   * @param rows Number of rows in the grid.
   * @param cols Number of columns in the grid.
   * @param resolution The resolution of the grid map.
   */
  GridMap(int rows, int cols, float resolution);

  inline unsigned char& at(Eigen::Vector2f point) {
    Eigen::Vector2i idx = worldToGrid(point);
    return Grid_::at(idx.y(), idx.x());
  }

  inline const unsigned char& at(Eigen::Vector2f point) const {
    Eigen::Vector2i idx = worldToGrid(point);
    return Grid_::at(idx.y(), idx.x());
  }

  inline bool inside(Eigen::Vector2f point) const {
    Eigen::Vector2i idx = worldToGrid(point);
    return Grid_::inside(idx.y(), idx.x());
  }

  inline bool isOccupied(unsigned int row, unsigned int col) const {
    return Grid_::at(row, col) == OCCUPIED;
  }

  /**
   * @brief Scans a ray in the grid map.
   *
   * @param origin The origin of the ray.
   * @param direction The direction of the ray.
   * @param max_range The maximum range of the ray.
   * @return The range of the ray.
   */
  float scanRay(const Eigen::Vector2f& origin, const Eigen::Vector2f& direction,
                const float max_range) const;

  /**
   * @brief Loads the grid map from an OccupancyGrid message.
   *
   * @param map The OccupancyGrid message.
   */
  void loadFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map);

  /**
   * @brief Loads the grid map from an image file.
   *
   * @param filename The image file name.
   * @param res The resolution of the grid map.
   */
  void loadFromImage(const char* filename, float resolution);

  /**
   * @brief Draws the grid map.
   *
   * @param dest The canvas where the grid map will be drawn.
   */
  void draw(Canvas& dest) const;
};