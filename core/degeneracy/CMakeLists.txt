# degeneracy: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_degeneracy INTERFACE)
add_library(mss::degeneracy ALIAS mss_degeneracy)
target_link_libraries(mss_degeneracy INTERFACE mss::common)
