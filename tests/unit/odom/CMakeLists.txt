# odom unit tests (O-01, O-02 and WheelIntegrator).
mss_add_test(test_odom test_wheel_integrator.cpp test_odom_residual.cpp ../alloc_counter_impl.cpp)
target_include_directories(test_odom PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
