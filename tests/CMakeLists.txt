find_package(GTest REQUIRED)
include(GoogleTest)

# EIGEN_RUNTIME_NO_MALLOC + -UNDEBUG: EigenMallocGuard (tests/unit/alloc_counter.hpp)
# makes any Eigen heap allocation in test code fail an eigen_assert. eigen_assert is
# compiled out under NDEBUG (Release), so test targets undefine it; this also enables
# assert() and Eigen's index checks in test code only, never in core/ or bench.
function(mss_add_test name)
  add_executable(${name} ${ARGN})
  mss_set_target_defaults(${name})
  target_link_libraries(${name} PRIVATE mss::pipeline mss::io GTest::gtest GTest::gtest_main)
  target_compile_definitions(${name} PRIVATE MSS_REPO_DIR="${CMAKE_SOURCE_DIR}")
  target_compile_definitions(${name} PRIVATE MSS_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
  target_compile_definitions(${name} PRIVATE EIGEN_RUNTIME_NO_MALLOC)
  target_compile_options(${name} PRIVATE -UNDEBUG)
  gtest_discover_tests(${name})
endfunction()

add_subdirectory(unit)
if(MSS_WITH_GTSAM)
  add_subdirectory(unit/backend)
endif()
add_subdirectory(component)
# integration/ gets its own CMakeLists when the first test lands.
