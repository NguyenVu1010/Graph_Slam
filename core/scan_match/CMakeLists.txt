# scan_match: point-to-plane scan-to-map registration (docs/derivations/scan_match.md).
add_library(mss_scan_match STATIC src/point_to_plane.cpp src/plane_fit.cpp src/scan_matcher.cpp)
add_library(mss::scan_match ALIAS mss_scan_match)
mss_set_target_defaults(mss_scan_match)
target_include_directories(mss_scan_match PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_scan_match PUBLIC mss::common mss::map mss::degeneracy)  # VoxelMap, DegeneracyConfig
