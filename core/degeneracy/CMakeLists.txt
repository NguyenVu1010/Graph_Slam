# degeneracy: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_degeneracy INTERFACE)
add_library(mss::degeneracy ALIAS mss_degeneracy)
target_include_directories(mss_degeneracy INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_degeneracy INTERFACE mss::common)
