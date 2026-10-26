# gnss: WGS-84 geodesy, fix gating, chi-square innovation gate and the w->e aligner.
add_library(mss_gnss STATIC src/geodesy.cpp src/gate.cpp src/innovation.cpp)
add_library(mss::gnss ALIAS mss_gnss)
mss_set_target_defaults(mss_gnss)
target_include_directories(mss_gnss PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_gnss PUBLIC mss::common)
