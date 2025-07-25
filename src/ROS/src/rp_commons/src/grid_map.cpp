#include "rp_commons/grid_map.h"

GridMap::GridMap(int rows, int cols, float resolution)
    : Grid_<uint8_t>(rows, cols) {
  _resolution = resolution;
}

float GridMap::scanRay(const Eigen::Vector2f& origin,
                       const Eigen::Vector2f& direction,
                       const float max_range) const {
  float range = 0;

  while (range < max_range) {
    // Calculate the current endpoint in the grid
    Eigen::Vector2i grid_endpoint = worldToGrid(origin + direction * range);
    int r = grid_endpoint.y();
    int c = grid_endpoint.x();
    // Check if the grid coordinates are inside the grid
    if (!Grid_::inside(r, c)) return max_range;
    // Check if the cell is occupied
    if (isOccupied(r, c)) return range;
    // Increment the range by the resolution
    range += _resolution;
  }

  return max_range;
}

void GridMap::loadFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map) {
  _rows = map.info.height;
  _cols = map.info.width;
  _cells.resize(_rows * _cols);
  _origin =
      Eigen::Vector2f(map.info.origin.position.x, map.info.origin.position.y);
  _resolution = map.info.resolution;
  for (unsigned int r = 0; r < _rows; ++r) {
    for (unsigned int c = 0; c < _cols; ++c) {
      // Calculate the index in the OccupancyGrid data array
      unsigned int i = _cols * (_rows - r - 1) + c;
      unsigned char value = map.data[i];
      // Set the value in the grid
      Grid_::at(r, c) = value;
    }
  }
}

void GridMap::loadFromImage(const char* filename, float res) {
  _resolution = res;
  std::cout << "loading [" << filename << "]" << std::endl;
  cv::Mat m = cv::imread(filename, cv::IMREAD_UNCHANGED);
  if (m.rows == 0 || m.channels() != 4) {
    throw std::runtime_error(
        "unable to load image or image does not have 4 channels");
  }
  cv::Mat loaded_image;
  cv::cvtColor(m, loaded_image, cv::COLOR_BGRA2GRAY);
  Grid_::resize(loaded_image.rows, loaded_image.cols);
  GridMapping::_origin =
      Eigen::Vector2f(0.0, -static_cast<float>(_rows) * _resolution);

  for (int r = 0; r < m.rows; ++r) {
    for (int c = 0; c < m.cols; ++c) {
      // Read the grayscale value from the loaded image
      uint8_t grayscale_value = loaded_image.at<uint8_t>(r, c);
      // Read the transparency (alpha channel) from the original image
      uint8_t alpha_value = m.at<cv::Vec4b>(r, c)[3];
      // Set the grid cell value based on the alpha and grayscale values
      if (alpha_value < 127) {
        Grid_::at(r, c) = UNKNOWN;
      } else if (grayscale_value < 147) {
        Grid_::at(r, c) = OCCUPIED;
      } else {
        Grid_::at(r, c) = FREE;
      }
    }
  }
  std::cout << "End load!!" << std::endl;
}

void GridMap::draw(Canvas& dest) const {
  drawGrid(dest, *this, FREE, OCCUPIED, UNKNOWN);
}
