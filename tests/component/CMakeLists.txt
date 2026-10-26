# Component tests: modules and binaries on synthetic data with a known answer.
mss_add_test(test_voxel_map_component test_voxel_map_component.cpp)
mss_add_test(test_gnss_component test_gnss_component.cpp)
target_include_directories(test_gnss_component PRIVATE ${CMAKE_SOURCE_DIR}/core/gnss/src)
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

# scan_match: point-to-plane registration on synthetic room / corridor (known pose).
mss_add_test(test_scan_match_component test_scan_match_component.cpp
  ${CMAKE_SOURCE_DIR}/tests/unit/alloc_counter_impl.cpp)
target_include_directories(test_scan_match_component PRIVATE ${CMAKE_SOURCE_DIR}/tests/unit)

# pipeline: estimator core on the synthetic box world (known trajectory), no-alloc, robustness.
mss_add_test(test_pipeline_component test_pipeline_component.cpp
  ${CMAKE_SOURCE_DIR}/tests/unit/alloc_counter_impl.cpp)
target_include_directories(test_pipeline_component PRIVATE
  ${CMAKE_SOURCE_DIR}/tests/unit ${CMAKE_SOURCE_DIR}/core/pipeline/src)
target_compile_definitions(test_pipeline_component PRIVATE
  MSS_TEST_WITH_GTSAM=$<BOOL:${MSS_WITH_GTSAM}>)
