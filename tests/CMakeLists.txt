find_package(GTest REQUIRED)
include(GoogleTest)

function(mss_add_test name)
  add_executable(${name} ${ARGN})
  mss_set_target_defaults(${name})
  target_link_libraries(${name} PRIVATE mss::pipeline mss::io GTest::gtest GTest::gtest_main)
  target_compile_definitions(${name} PRIVATE MSS_REPO_DIR="${CMAKE_SOURCE_DIR}")
  gtest_discover_tests(${name})
endfunction()

add_subdirectory(unit)
if(MSS_WITH_GTSAM)
  add_subdirectory(unit/backend)
endif()
# component/ and integration/ get their own CMakeLists when the first test lands.
