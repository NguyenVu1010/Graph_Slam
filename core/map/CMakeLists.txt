# map: voxel hash map of LiDAR points in w (ADR-007).
add_library(mss_map STATIC src/voxel_map.cpp)
add_library(mss::map ALIAS mss_map)
mss_set_target_defaults(mss_map)
target_include_directories(mss_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_map PUBLIC mss::common)
