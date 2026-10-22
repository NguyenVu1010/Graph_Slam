# odom: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_odom INTERFACE)
add_library(mss::odom ALIAS mss_odom)
target_include_directories(mss_odom INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_odom INTERFACE mss::common)
