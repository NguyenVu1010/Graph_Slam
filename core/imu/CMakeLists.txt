# imu: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_imu INTERFACE)
add_library(mss::imu ALIAS mss_imu)
target_link_libraries(mss_imu INTERFACE mss::common)
