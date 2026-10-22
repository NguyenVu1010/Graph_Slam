cmake_minimum_required(VERSION 3.22)
project(mss_ros2 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(mss_warnings)

find_package(ament_cmake REQUIRED)
find_package(mss_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mss_node src/main.cpp src/rclcpp_logger.cpp)
mss_set_target_defaults(mss_node)
target_include_directories(mss_node PRIVATE include)
target_link_libraries(mss_node PRIVATE mss::pipeline)
ament_target_dependencies(mss_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS mss_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES ../config/default.yaml DESTINATION share/${PROJECT_NAME}/config)

ament_package()
