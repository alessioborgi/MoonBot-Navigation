cmake_minimum_required(VERSION 3.8)
project(rp_localizer)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rp_commons REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} src/localizer.cpp src/node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rp_commons geometry_msgs nav_msgs sensor_msgs tf2_ros)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(localizer_node src/localizer_node.cpp)
ament_target_dependencies(localizer_node rclcpp rp_commons geometry_msgs nav_msgs sensor_msgs tf2_ros)
target_link_libraries(localizer_node ${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rp_commons geometry_msgs nav_msgs sensor_msgs tf2_ros)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the executable
install(TARGETS localizer_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()