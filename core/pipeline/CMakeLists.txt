# pipeline: owns threads and bounded queues; the only module apps link against.
add_library(mss_pipeline INTERFACE)
add_library(mss::pipeline ALIAS mss_pipeline)
target_link_libraries(mss_pipeline INTERFACE
  mss::common mss::preprocess mss::map mss::scan_match mss::degeneracy
  mss::imu mss::gnss mss::odom mss::loop mss::backend)
