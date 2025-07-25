#include "rp_controller/differential_drive_controller.h"

#include <iostream>

void DiffDriveController::update(const Eigen::Isometry2f& current_pose) {
  if (_done) {
    return;
  }

  _max_v = 1;
  _k_rho = 0.5;

  _k_yaw = 0.7;
  _kd_yaw = 0.2;

  std::cout << "Current pose: " << current_pose.translation().x() << " "
            << current_pose.translation().y() << " " << atan2(current_pose.rotation()(1, 0), current_pose.rotation()(0, 0)) << std::endl;

  // const auto& target_pose_in_world = // TODO: Get the target pose in the world frame;
  const auto& target_pose_in_world = _waypoints[_current_waypoint];
  // const auto& target_pose_in_robot = // TODO: Get the target pose in the robot frame;
  const auto target_pose_in_robot = current_pose.inverse() * target_pose_in_world;
  
  Eigen::Vector2f delta_pos = Eigen::Vector2f::Zero();
  // delta_pos = // TODO: Compute the delta in translation;
  delta_pos = target_pose_in_robot.translation();
  // const auto delta_yaw = // TODO: Compute the delta in rotation (atan2);
  // const auto delta_yaw = atan2(target_pose_in_robot.rotation()(1, 0), target_pose_in_robot.rotation()(0, 0));
  const auto delta_yaw = atan2(delta_pos.y(), delta_pos.x());
  std::cout << "curr waypoint: " << target_pose_in_world.translation().x() << " " <<  target_pose_in_world.translation().y() << std::endl;
  std::cout << "delta pos: " << delta_pos.x() << " " << delta_pos.y() << ", delta yaw: " << delta_yaw << std::endl << std::endl;

  // current errors
  float rho_err = delta_pos.norm();
  float yaw_err = delta_yaw;

  // derivative terms
  float rho_dot = (rho_err - _prev_rho_err) / _dt;
  float yaw_dot = (yaw_err - _prev_yaw_err) / _dt;

  // PD outputs
  // zero‐out linear if the target is “behind” by more than 90°
  if (abs(yaw_err) > M_PI_4) {
    _output_v = 0.0f;
  } else {
    _output_v = _k_rho * rho_err + _kd_rho * rho_dot;
    if (_output_v > _max_v)
      _output_v = _max_v; 
  }

  _output_w = _k_yaw * yaw_err + _kd_yaw * yaw_dot;
  if (_output_w > _max_w)
    _output_w = _max_w; 

  // store for next iteration
  _prev_rho_err = rho_err;
  _prev_yaw_err = yaw_err;


  //***************  Begin of obstacle avoidance procedure ***************//
  if (false) {
    // HINTS:
    // - Use the laser measurements to avoid obstacles
    // - Activate the obstacle avoidance when the robot is close to an obstacle
    // - Check if there are more points on the left or on the right (the azimuth of a point in the laser scan frame is atan2(y, x))
    // - If there are more points on the right, turn left and vice versa
    // - The rotational velocity should be proportional to the angle to the closest obstacle (0 when the obstacle is at +/-3.14 radians)

    // Note: The above HINTS are taken from our implementation, you can come up with your own solution

    auto& obstacles = _measurements;
    const float obs_threshold = 0.5f;
    const float max_w = 4.0f;

    int count_left = 0;
    Eigen::Vector2f closest = Eigen::Vector2f(-100, 100);
    for (const auto& obs : obstacles) {
      if (obs.norm() < obs_threshold) {
        if (obs.norm() < closest.norm()) {
          closest = obs;
        }
        if (obs.y() > 0) {
          count_left++;
        } else {
          count_left--;
        }
      }
    }

    if (closest.x() > 0 && ((closest.y() > 0 && delta_pos.y() > 0) || (closest.y() < 0 && delta_pos.y() < 0))) {

      double angle = atan2(closest.y(), closest.x());
      std::cout << "Closest obstacle: " << closest.x() << " " << closest.y() << ": " << closest.norm() << ", " << angle << std::endl;

      if (angle > 0) angle = -angle;
      // -pi/2 -> 0   ::   0 -> max_w
      _output_w = ((angle+M_PI_2) / M_PI_2) * max_w;

      if (count_left > 0) {
        _output_w = -_output_w;
      }

      if (_output_v == 0) {
        _output_v = 0.5f;
      }

      // if next waypoint is closer than the current one, skip the current one
      if (_current_waypoint < _waypoints.size()-1) {
        auto next_wp = _waypoints[_current_waypoint+1];
        auto next_wp_in_robot = current_pose.inverse() * next_wp;
        auto next_delta_pos = next_wp_in_robot.translation();
        if (next_delta_pos.norm() < delta_pos.norm()) {
          _current_waypoint++;
          // _output_v = 0.0f;
          // _output_w = 0.0f;

          // instead of blocking the robot, just update the velocities with new waypoint
          update(current_pose);
          return;
        }
      }

    }

  }

  //***************  End of obstacle avoidance procedure ***************//

  // Check if the robot is close to the target waypoint
  while (delta_pos.norm() < _tolerance) {
    
    // if its the last waypoint i dont want to skip it
    if (_current_waypoint == _waypoints.size()-1)
      break;

    // Move to the next waypoint
    _current_waypoint++;
    std::cerr << "New waypoint" << std::endl;
    if (_current_waypoint >= _waypoints.size()) {
      std::cerr << "Completed path" << std::endl;
      _done = true;
      _output_v = 0.0f;
      _output_w = 0.0f;
    }

    const auto& target_p_in_w = _waypoints[_current_waypoint];
    const auto target_p_in_r = current_pose.inverse() * target_p_in_w;
    delta_pos = Eigen::Vector2f::Zero();
    delta_pos = target_p_in_r.translation();
    std::cerr << delta_pos.norm() << std::endl;
  }

  return;
}
