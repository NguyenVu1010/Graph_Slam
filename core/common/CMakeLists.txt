add_library(mss_common STATIC src/logger.cpp src/lie.cpp src/scan_frame.cpp)
add_library(mss::common ALIAS mss_common)
mss_set_target_defaults(mss_common)
target_include_directories(mss_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_common PUBLIC Eigen3::Eigen Sophus::Sophus)
# Sophus is built without fmt: use its basic logging (same in tools/wsl/setup.sh, CI, docker).
target_compile_definitions(mss_common PUBLIC SOPHUS_USE_BASIC_LOGGING)
