# map: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_map INTERFACE)
add_library(mss::map ALIAS mss_map)
target_link_libraries(mss_map INTERFACE mss::common)
