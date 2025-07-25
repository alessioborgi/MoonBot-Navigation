cmake_minimum_required(VERSION 3.8)
project(rp_map_server)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rp_commons REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(${PROJECT_NAME} src/node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rp_commons nav_msgs)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add the executable
add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(map_server_node ${PROJECT_NAME})

# Link against the necessary libraries
ament_target_dependencies(map_server_node rp_commons rclcpp nav_msgs)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rp_commons nav_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the executable
install(TARGETS map_server_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()