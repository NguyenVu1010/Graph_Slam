# 2D SLAM for a single-core MCU without an OS (ADR-034). Standalone: not part of the root
# build, links nothing from core/. This is the host build (tests, CI).
cmake_minimum_required(VERSION 3.22)
project(mcu2d CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mss2d src/config.cpp src/merger.cpp src/occupancy_grid.cpp src/distance_field.cpp
                  src/degeneracy.cpp src/scan_matcher.cpp src/motion.cpp src/frontend.cpp)
target_include_directories(mss2d PUBLIC include)
target_compile_options(mss2d PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-exceptions -fno-rtti)

enable_testing()
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
add_executable(mss2d_tests tests/ring_test.cpp tests/merger_test.cpp tests/config_test.cpp
                           tests/no_alloc_test.cpp tests/pose2_test.cpp tests/grid_test.cpp
                           tests/degeneracy_test.cpp tests/matcher_test.cpp)
target_link_libraries(mss2d_tests PRIVATE mss2d GTest::gtest_main Threads::Threads)
target_compile_options(mss2d_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME mss2d_tests COMMAND mss2d_tests)
