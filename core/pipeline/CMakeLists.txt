# pipeline: owns threads and bounded queues; the only module apps link against.
# The only place that parses YAML (ADR-008): yaml-cpp stays PRIVATE.
add_library(mss_pipeline STATIC src/config.cpp)
add_library(mss::pipeline ALIAS mss_pipeline)
mss_set_target_defaults(mss_pipeline)
target_include_directories(mss_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_pipeline PUBLIC
  mss::common mss::preprocess mss::map mss::scan_match mss::degeneracy
  mss::imu mss::gnss mss::odom mss::loop mss::backend)
target_link_libraries(mss_pipeline PRIVATE yaml-cpp::yaml-cpp)
