# Desktop / Pi5 superbuild without ROS: core library, unit tests and bench.
# colcon users build core/ and ros2_node/ as packages instead (see README).
cmake_minimum_required(VERSION 3.22)
project(multi_sensor_slam LANGUAGES CXX)

option(MSS_BUILD_TESTS "Build unit/component tests" ON)
option(MSS_BUILD_BENCH "Build the latency benchmark binary" ON)

add_subdirectory(core)
if(MSS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
if(MSS_BUILD_BENCH)
  add_subdirectory(bench)
endif()
