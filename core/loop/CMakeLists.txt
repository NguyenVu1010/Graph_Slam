# loop: loop-closure detection (radius search + Scan Context yaw) and ICP verification
# (ADR-026, docs/derivations/loop_closure.md). Reuses scan_match::ScanMatcher and
# map::VoxelMap; no GTSAM.
add_library(mss_loop STATIC src/loop_detector.cpp)
add_library(mss::loop ALIAS mss_loop)
mss_set_target_defaults(mss_loop)
target_include_directories(mss_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_loop PUBLIC mss::common mss::scan_match mss::map)
