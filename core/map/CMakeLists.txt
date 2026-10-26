# map: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_map INTERFACE)
add_library(mss::map ALIAS mss_map)
target_include_directories(mss_map INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_map INTERFACE mss::common)
