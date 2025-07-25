cmake_minimum_required(VERSION 3.8)
project(rp_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Werror")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rp_commons REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/differential_drive_controller.cpp
    src/node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} rclcpp rp_commons geometry_msgs nav_msgs tf2 tf2_ros)

add_executable(differential_drive_controller_node src/diffdrive_controller.cpp)
ament_target_dependencies(differential_drive_controller_node rclcpp rp_commons geometry_msgs nav_msgs tf2 tf2_ros)
target_link_libraries(differential_drive_controller_node ${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rp_commons geometry_msgs nav_msgs tf2 tf2_ros)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the executable
install(TARGETS differential_drive_controller_node
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()