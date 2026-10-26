# loop: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_loop INTERFACE)
add_library(mss::loop ALIAS mss_loop)
target_include_directories(mss_loop INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_loop INTERFACE mss::common)
