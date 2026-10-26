# pipeline: owns threads and bounded queues; the only module apps link against.
# Parses the pipeline config (ADR-008; `io` parses `index.json` with yaml-cpp):
# yaml-cpp stays PRIVATE.
add_library(mss_pipeline STATIC
  src/config.cpp
  src/estimator.cpp
  src/estimator_core.cpp
  src/frames.cpp
  src/gnss_fusion.cpp
  src/imu_buffer.cpp
  src/imu_trajectory_builder.cpp
  src/keyframe_policy.cpp
  src/keyframe_store.cpp)
add_library(mss::pipeline ALIAS mss_pipeline)
mss_set_target_defaults(mss_pipeline)
target_include_directories(mss_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_pipeline PUBLIC
  mss::common mss::preprocess mss::map mss::scan_match mss::degeneracy
  mss::imu mss::gnss mss::odom mss::loop mss::backend)
target_link_libraries(mss_pipeline PRIVATE yaml-cpp::yaml-cpp)
