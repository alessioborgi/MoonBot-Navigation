#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "rp_commons/grid_map.h"
#include "rp_simulator/laser_scanner.h"
#include "rp_simulator/unicycle.h"
#include "rp_simulator/world.h"
#include "rp_simulator/world_item.h"

Eigen::Isometry2f fromCoefficients(float tx, float ty, float alpha) {
  Eigen::Isometry2f iso;
  iso.setIdentity();
  iso.translation() << tx, ty;
  iso.linear() = Eigen::Rotation2Df(alpha).matrix();
  return iso;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("simulator_node");

  std::string config_file;
  node->declare_parameter<std::string>("config_file");
  if (!node->has_parameter("config_file")) {
    RCLCPP_ERROR(node->get_logger(),
                 "Parameter 'config_file' is required but not set.");
    rclcpp::shutdown();
    throw std::runtime_error(
        "Parameter 'config_file' is required but not set.");
  }

  // Get the parameter value
  node->get_parameter("config_file", config_file);

  YAML::Node config = YAML::LoadFile(config_file);
  std::string image_filename = config["image"].as<std::string>();
  float resolution = config["resolution"].as<float>();
  bool publish_robots_gt = config["publish_robots_gt"].as<bool>();

  // Extract prefix from the YAML file path and append the image filename
  std::filesystem::path config_path(config_file);
  std::filesystem::path prefix = config_path.parent_path();
  std::filesystem::path image_path = prefix / image_filename;
  std::string image_path_str = image_path.string();

  std::cout << "Running " << argv[0] << " with configuration" << std::endl
            << "-image path: " << image_path_str << std::endl
            << "-resolution: " << resolution << std::endl;

  GridMap grid_map;
  grid_map.loadFromImage(image_path_str.c_str(), resolution);

  Canvas canvas;
  grid_map.draw(canvas);

  // Create the WorldNode, which will manage ROS 2 interactions and simulation
  auto world_object = std::make_shared<World>(grid_map, node, 0.1f);

  std::vector<std::shared_ptr<UnicyclePlatform>> robots;
  std::vector<std::shared_ptr<LaserScanner>> scanners;

  for (const auto& robot_config : config["robots"]) {
    std::string robot_name = robot_config["name"].as<std::string>();
    std::vector<float> robot_in_world =
        robot_config["robot_in_world"].as<std::vector<float>>();
    float robot_radius = robot_config["robot_radius"].as<float>();

    auto robot = std::make_shared<UnicyclePlatform>(
        *world_object, robot_name, node,
        fromCoefficients(robot_in_world[0], robot_in_world[1],
                         robot_in_world[2]),
        publish_robots_gt);
    robot->_radius = robot_radius;
    robots.push_back(robot);

    for (const auto& scanner_config : robot_config["scanners"]) {
      std::string scanner_name = scanner_config["name"].as<std::string>();
      std::vector<float> scanner_in_robot =
          scanner_config["scanner_in_robot"].as<std::vector<float>>();
      float scanner_radius = scanner_config["scanner_radius"].as<float>();
      float scanner_freq = scanner_config["scanner_freq"].as<float>();

      auto scanner = std::make_shared<LaserScanner>(
          *robot, scanner_name, node,
          fromCoefficients(scanner_in_robot[0], scanner_in_robot[1],
                           scanner_in_robot[2]),
          scanner_freq);
      scanner->_radius = scanner_radius;
      scanners.push_back(scanner);
    }
  }

  // Use the ROS 2 multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(world_object->_node);

  // Simulation loop
  while (rclcpp::ok()) {
    // std::cout << "Tick" << std::endl;
    world_object->draw(canvas);
    showCanvas(canvas, 10);

    // Process ROS 2 callbacks
    executor.spin_some();
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}