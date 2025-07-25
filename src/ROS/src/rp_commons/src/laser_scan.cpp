#include "rp_commons/laser_scan.h"

LaserScan::LaserScan(float range_min, float range_max, float angle_min,
                     float angle_max, int ranges_num) {
  _range_min = range_min;
  _range_max = range_max;
  _angle_min = angle_min;
  _angle_max = angle_max;
  _ranges_num = ranges_num;
  _ranges.resize(ranges_num);
  _angle_increment = (angle_max - angle_min) / float(ranges_num);
  std::fill(_ranges.begin(), _ranges.end(), _range_max);
}

std::vector<Eigen::Vector2f> LaserScan::toCartesian() const {
  std::vector<Eigen::Vector2f> points;
  for (unsigned int i = 0; i < _ranges.size(); ++i) {
    float beam_angle = _angle_min + _angle_increment * i;
    Eigen::Vector2f d(cos(beam_angle) * _ranges[i],
                      sin(beam_angle) * _ranges[i]);
    points.push_back(d);
  }
  return points;
}

void LaserScan::draw(Canvas& canevasso, const GridMap& grid_map,
                     const Eigen::Isometry2f& pose) const {
  Eigen::Vector2i center_px = grid_map.worldToGrid(pose.translation());

  for (size_t i = 0; i < _ranges.size(); ++i) {
    float beam_angle = _angle_min + _angle_increment * i;
    Eigen::Vector2f d(cos(beam_angle) * _ranges[i],
                      sin(beam_angle) * _ranges[i]);
    Eigen::Vector2f ep = pose * d;
    Eigen::Vector2i ep_px = grid_map.worldToGrid(ep);
    drawLine(canevasso, center_px.cast<int>(), ep_px.cast<int>(), 90);
  }
}
