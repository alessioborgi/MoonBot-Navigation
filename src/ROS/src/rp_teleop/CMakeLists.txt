cmake_minimum_required(VERSION 3.8)
project(rp_teleop)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Export compile commands for development tools like clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the build type to Release
set(CMAKE_BUILD_TYPE Release)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(teleop_node src/teleop_node.cpp)

# Link against the necessary libraries
ament_target_dependencies(teleop_node rclcpp geometry_msgs)

# Install the executable
install(TARGETS teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()