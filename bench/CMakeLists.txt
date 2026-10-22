add_executable(mss_bench main.cpp)
mss_set_target_defaults(mss_bench)
target_link_libraries(mss_bench PRIVATE mss::pipeline)
install(TARGETS mss_bench RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
