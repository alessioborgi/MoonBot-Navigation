#include "rp_map_server/node.h"

MapServerNode::MapServerNode() : Node("map_server_node") {
  // Declare the parameters
  this->declare_parameter<std::string>("image");
  this->declare_parameter<float>("resolution");

  // Check if the parameters are set
  if (!this->has_parameter("image") || !this->has_parameter("resolution")) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Parameters 'image' and 'resolution' are required but not set.");
    rclcpp::shutdown();
    throw std::runtime_error(
        "Parameters 'image' and 'resolution' are required but not set.");
  }

  // Get the parameter values
  std::string image_path = "";
  float resolution = 0.1;
  this->get_parameter("image", image_path);
  this->get_parameter("resolution", resolution);

  // Load the grid map from the image
  _grid_map.loadFromImage(image_path.c_str(), resolution);

  // Create a publisher for the occupancy grid map
  _occupancy_grid_pub =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

  // Create a timer inside World for publishing the map
  _timer_map =
      this->create_wall_timer(std::chrono::duration<float>(_map_interval),
                              std::bind(&MapServerNode::publishMap, this));
}

/**
 * @brief Publishes the occupancy grid map.
 *
 * This function is called periodically by the timer to publish the current
 * state of the occupancy grid map.
 */
void MapServerNode::publishMap() {
  // Create an OccupancyGrid message and fill it with the grid map data
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = this->now();
  occupancy_grid.header.frame_id = "map";

  occupancy_grid.info.map_load_time = this->now();
  occupancy_grid.info.resolution = _grid_map._resolution;
  occupancy_grid.info.width = _grid_map._cols;
  occupancy_grid.info.height = _grid_map._rows;

  // Set the origin position of the grid map
  occupancy_grid.info.origin.position.x = _grid_map._origin.x();
  occupancy_grid.info.origin.position.y = _grid_map._origin.y();

  // Resize the data vector to fit the grid map dimensions
  occupancy_grid.data.resize(_grid_map._rows * _grid_map._cols);

  // Fill the occupancy grid data
  for (size_t r = 0; r < _grid_map._rows; ++r) {
    for (size_t c = 0; c < _grid_map._cols; ++c) {
      uint8_t grayscale_value =
          _grid_map.at(r, c);  // Get the grayscale value at the current cell

      // Calculate the index in the occupancy grid data vector
      size_t index =
          occupancy_grid.info.width * (occupancy_grid.info.height - r - 1) + c;

      // Convert the grayscale value to occupancy grid value
      switch (grayscale_value) {
        case GridMap::FREE:
          occupancy_grid.data[index] = 0;  // Free cell
          break;
        case GridMap::OCCUPIED:
          occupancy_grid.data[index] = 100;  // Occupied cell
          break;
        case GridMap::UNKNOWN:
          occupancy_grid.data[index] = -1;  // Unknown cell
          break;
        default:
          occupancy_grid.data[index] = -1;  // Default to unknown cell
          break;
      }
    }
  }

  _occupancy_grid_pub->publish(occupancy_grid);  // Publish the occupancy grid
}