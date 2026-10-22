find_package(GTest REQUIRED)
include(GoogleTest)

function(mss_add_test name)
  add_executable(${name} ${ARGN})
  mss_set_target_defaults(${name})
  target_link_libraries(${name} PRIVATE mss::pipeline GTest::gtest GTest::gtest_main)
  gtest_discover_tests(${name})
endfunction()

add_subdirectory(unit)
# component/ and integration/ get their own CMakeLists when the first test lands.
