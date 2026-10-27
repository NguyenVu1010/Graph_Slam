# Tests that link GTSAM. Added only when MSS_WITH_GTSAM is ON.
# Imported targets are directory-scoped: core/ found GTSAM, so find it again here.
find_package(GTSAM REQUIRED)
mss_add_test(test_lie_gtsam test_lie_gtsam.cpp)
target_link_libraries(test_lie_gtsam PRIVATE gtsam)
mss_add_test(test_imu_preint_gtsam test_imu_preint_gtsam.cpp)
target_link_libraries(test_imu_preint_gtsam PRIVATE gtsam)

# Tests of core/backend internals: link GTSAM and see core/backend/src (internal headers).
function(mss_add_backend_test name)
  mss_add_test(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE gtsam)
  target_include_directories(${name} PRIVATE
    ${CMAKE_SOURCE_DIR}/core/backend/src ${CMAKE_SOURCE_DIR}/tests/unit)
endfunction()
mss_add_backend_test(test_backend_conversions test_backend_conversions.cpp)
mss_add_backend_test(test_backend_factors test_backend_factors.cpp)
mss_add_backend_test(test_backend_graph test_backend_graph.cpp)
mss_add_backend_test(test_backend_isam test_backend_isam.cpp)
# ADR-018: the test reads tbb::global_control directly.
find_package(TBB REQUIRED)
mss_add_backend_test(test_backend_determinism test_backend_determinism.cpp)
target_link_libraries(test_backend_determinism PRIVATE TBB::tbb)
