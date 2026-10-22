# loop: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_loop INTERFACE)
add_library(mss::loop ALIAS mss_loop)
target_link_libraries(mss_loop INTERFACE mss::common)
