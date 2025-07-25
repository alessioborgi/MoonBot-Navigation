cmake_minimum_required(VERSION 3.8)
project(rp_simulator)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rp_commons REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/world_item.cpp
  src/laser_scanner.cpp
  src/unicycle.cpp)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rp_commons sensor_msgs geometry_msgs nav_msgs tf2_ros rclcpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(simulator src/simulator.cpp)
ament_target_dependencies(simulator rp_commons rclcpp)
target_link_libraries(simulator ${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rp_commons sensor_msgs geometry_msgs nav_msgs tf2_ros rclcpp)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the executable
install(TARGETS simulator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()