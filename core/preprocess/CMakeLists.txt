# preprocess: switch to STATIC and add sources when the first .cpp lands.
add_library(mss_preprocess INTERFACE)
add_library(mss::preprocess ALIAS mss_preprocess)
target_link_libraries(mss_preprocess INTERFACE mss::common)
