# imu: on-manifold preintegration and static initialisation
# (docs/derivations/imu_preintegration.md). No GTSAM, no ROS.
add_library(mss_imu STATIC src/preintegrator.cpp)
add_library(mss::imu ALIAS mss_imu)
mss_set_target_defaults(mss_imu)
target_include_directories(mss_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_imu PUBLIC mss::common)
