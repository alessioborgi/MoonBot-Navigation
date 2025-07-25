#pragma once
#include <Eigen/Dense>
#include <vector>

class DiffDriveController {
 public:
  DiffDriveController()
      : _max_v(10), // TODO: Tune the maximum linear velocity
        _max_w(2.0), // TODO: Tune the maximum angular velocity
        _tolerance(1), // TODO: Tune the tolerance
        _k_rho(10), // TODO: Tune the gain for the distance to the next waypoint
        _k_yaw(0.5), // TODO: Tune the gain for the angle to the next waypoint
        _done(true),
        _obstacle_avoidance(false) {}

  void update(const Eigen::Isometry2f& current_pose);

  inline void setWaypoints(const std::vector<Eigen::Isometry2f>& waypoints) {
    _waypoints = waypoints;
    _current_waypoint = 0;
    _done = false;
  }

  inline void setLaserMeasurements(
      const std::vector<Eigen::Vector2f> measurements) {
    _measurements = measurements;
  }

  inline void setObstacleAvoidance() { _obstacle_avoidance = true; }

  inline const float& getV() const { return _output_v; }
  inline const float& getW() const { return _output_w; }
  inline bool isDone() const { return _done; }

 protected:
  std::vector<Eigen::Isometry2f> _waypoints;
  unsigned int _current_waypoint = 0;
  std::vector<Eigen::Vector2f> _measurements;
  float _max_v, _max_w;
  float _tolerance;
  float _k_rho, _k_yaw;

  float _kd_rho    = 0.0f;           // derivative gain for linear
  float _kd_yaw    = 0.5f;           // derivative gain for angular
  float _prev_rho_err = 0.0f;        // last linear error
  float _prev_yaw_err = 0.0f;        // last angular error
  bool _done;
  bool _obstacle_avoidance;
  float _dt = 0.1f; // time step for derivative calculation

  float _output_v, _output_w;
};