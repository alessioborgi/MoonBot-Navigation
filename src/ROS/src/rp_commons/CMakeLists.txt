cmake_minimum_required(VERSION 3.8)
project(rp_commons)

add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(OpenCV 4.5 REQUIRED)


add_library(${PROJECT_NAME} SHARED
    src/grid_map.cpp
    src/distance_map.cpp
    src/laser_scan.cpp
    src/draw_helpers.cpp)

ament_target_dependencies(${PROJECT_NAME} PUBLIC nav_msgs sensor_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

add_executable(test_distance_map_node src/test_distance_map_node.cpp)
ament_target_dependencies(test_distance_map_node rclcpp nav_msgs)
target_link_libraries(test_distance_map_node ${PROJECT_NAME})

# Export Library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(nav_msgs sensor_msgs Eigen3)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install library
install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the executables
install(TARGETS test_distance_map_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()