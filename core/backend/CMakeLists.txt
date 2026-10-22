# backend: the ONLY module that includes GTSAM. Other modules return core types.
add_library(mss_backend INTERFACE)
add_library(mss::backend ALIAS mss_backend)
target_include_directories(mss_backend INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_backend INTERFACE mss::common)
if(MSS_WITH_GTSAM)
  target_link_libraries(mss_backend INTERFACE gtsam)
  target_compile_definitions(mss_backend INTERFACE MSS_WITH_GTSAM=1)
endif()
