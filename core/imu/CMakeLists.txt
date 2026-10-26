# imu: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_imu INTERFACE)
add_library(mss::imu ALIAS mss_imu)
target_include_directories(mss_imu INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_imu INTERFACE mss::common)
