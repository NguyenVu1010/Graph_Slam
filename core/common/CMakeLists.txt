add_library(mss_common STATIC src/logger.cpp)
add_library(mss::common ALIAS mss_common)
mss_set_target_defaults(mss_common)
target_include_directories(mss_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_common PUBLIC Eigen3::Eigen)
