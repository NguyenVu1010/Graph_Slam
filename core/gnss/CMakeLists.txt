# gnss: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_gnss INTERFACE)
add_library(mss::gnss ALIAS mss_gnss)
target_include_directories(mss_gnss INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_gnss INTERFACE mss::common)
