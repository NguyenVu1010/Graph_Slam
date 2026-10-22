# odom: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_odom INTERFACE)
add_library(mss::odom ALIAS mss_odom)
target_link_libraries(mss_odom INTERFACE mss::common)
