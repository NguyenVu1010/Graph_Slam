# preprocess: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_preprocess INTERFACE)
add_library(mss::preprocess ALIAS mss_preprocess)
target_include_directories(mss_preprocess INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_preprocess INTERFACE mss::common mss::imu)  # ImuTrajectory
