# Component tests: modules and binaries on synthetic data with a known answer.
mss_add_test(test_voxel_map_component test_voxel_map_component.cpp)
if(MSS_BUILD_BENCH)
  mss_add_test(test_bench_replay test_bench_replay.cpp)
  target_compile_definitions(test_bench_replay PRIVATE MSS_BENCH_PATH="$<TARGET_FILE:mss_bench>")
  add_dependencies(test_bench_replay mss_bench)
endif()
mss_add_test(test_degeneracy_scenes test_degeneracy_scenes.cpp)
mss_add_test(test_odom_component test_odom_component.cpp)
mss_add_test(test_preprocess_component test_preprocess_deskew_plane.cpp)

# imu: static init + preintegration on a tilted circle (known answer).
mss_add_test(test_imu_component test_imu_component.cpp)
