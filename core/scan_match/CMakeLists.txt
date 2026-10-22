# scan_match: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_scan_match INTERFACE)
add_library(mss::scan_match ALIAS mss_scan_match)
target_include_directories(mss_scan_match INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_scan_match INTERFACE mss::common)
