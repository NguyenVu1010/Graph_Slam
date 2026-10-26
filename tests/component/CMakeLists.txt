# Component tests: modules and binaries on synthetic data with a known answer.
if(MSS_BUILD_BENCH)
  mss_add_test(test_bench_replay test_bench_replay.cpp)
  target_compile_definitions(test_bench_replay PRIVATE MSS_BENCH_PATH="$<TARGET_FILE:mss_bench>")
  add_dependencies(test_bench_replay mss_bench)
endif()
mss_add_test(test_degeneracy_scenes test_degeneracy_scenes.cpp)
