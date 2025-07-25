cmake_minimum_required(VERSION 3.8)
project(rp_eval)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Werror")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rp_commons REQUIRED)
find_package(rp_controller REQUIRED)
find_package(rp_simulator REQUIRED)
find_package(rp_planner REQUIRED)
find_package(rp_map_server REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(test_controller src/test_controller.cpp)
ament_target_dependencies(test_controller
    rclcpp
    rp_commons
    rp_controller
    rp_simulator
    rp_planner
    rp_map_server)

add_executable(test_controller_ca src/test_controller_ca.cpp)
ament_target_dependencies(test_controller_ca
    rclcpp
    rp_commons
    rp_controller
    rp_simulator
    rp_planner
    rp_map_server)

# Install the executable
install(TARGETS test_controller test_controller_ca
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()