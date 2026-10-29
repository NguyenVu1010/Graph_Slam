cmake_minimum_required(VERSION 3.22)
project(mss_ros2 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(mss_warnings)

find_package(ament_cmake REQUIRED)
find_package(mss_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Message <-> core struct conversions (ADR-024), no node state; unit-tested on its own.
add_library(mss_ros2_conversions STATIC src/conversions.cpp)
mss_set_target_defaults(mss_ros2_conversions)
target_include_directories(mss_ros2_conversions PUBLIC include)
target_link_libraries(mss_ros2_conversions PUBLIC mss::common mss::gnss)
# Keyword form: target_link_libraries above uses the keyword signature, and mixing the two is
# a CMake error. SYSTEM keeps -Werror from firing on ROS headers.
ament_target_dependencies(mss_ros2_conversions SYSTEM PUBLIC
  builtin_interfaces sensor_msgs nav_msgs geometry_msgs)

add_executable(mss_node src/main.cpp src/mss_node.cpp src/rclcpp_logger.cpp)
mss_set_target_defaults(mss_node)
target_include_directories(mss_node PRIVATE include)
target_link_libraries(mss_node PRIVATE mss_ros2_conversions mss::pipeline)
ament_target_dependencies(mss_node SYSTEM PUBLIC
  ament_index_cpp rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS mss_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES ../config/default.yaml config/bag_qos.yaml DESTINATION share/${PROJECT_NAME}/config)
# Scenario overlays (config/sim/*.yaml) so mss_bag.launch.py can name them as overlay:=sim/<x>.
install(DIRECTORY ../config/sim DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  mss_set_target_defaults(test_conversions)
  target_link_libraries(test_conversions mss_ros2_conversions)
endif()

ament_package()
