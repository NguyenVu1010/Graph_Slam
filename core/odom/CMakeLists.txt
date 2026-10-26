# odom: wheel-odometry integration (WheelIntegrator) and residuals (O-01, O-02).
# No GTSAM, no Sophus here: Lie operations come from mss/common/lie.hpp.
add_library(mss_odom STATIC src/wheel_integrator.cpp)
add_library(mss::odom ALIAS mss_odom)
mss_set_target_defaults(mss_odom)
target_include_directories(mss_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_odom PUBLIC mss::common)
