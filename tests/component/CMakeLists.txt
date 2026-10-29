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

# IMU-only observability study (docs/derivations/imu_only_observability.md): skipped unless
# MSS_IMU_STUDY=1; shares the pipeline helpers (pipeline_test_support.hpp).
mss_add_test(test_imu_only_study test_imu_only_study.cpp)
target_include_directories(test_imu_only_study PRIVATE ${CMAKE_SOURCE_DIR}/core/pipeline/src
  ${CMAKE_SOURCE_DIR}/core/backend/src)  # study_seams.hpp only (no GTSAM types)
target_compile_definitions(test_imu_only_study PRIVATE
  MSS_TEST_WITH_GTSAM=$<BOOL:${MSS_WITH_GTSAM}>)

# backend: estimate consistent with its marginal along an unobserved axis (PR 4e). GTSAM only.
if(MSS_WITH_GTSAM)
  mss_add_test(test_backend_consistency test_backend_consistency.cpp)
  target_include_directories(test_backend_consistency PRIVATE ${CMAKE_SOURCE_DIR}/tests/unit)
endif()

# GNSS corridor fusion diagnosis (docs/derivations/gnss_fusion_study.md section 13): skipped
# unless MSS_GNSS_CORRIDOR_STUDY=1.
mss_add_test(test_gnss_corridor_study test_gnss_corridor_study.cpp)
target_include_directories(test_gnss_corridor_study PRIVATE ${CMAKE_SOURCE_DIR}/core/pipeline/src)
target_compile_definitions(test_gnss_corridor_study PRIVATE
  MSS_TEST_WITH_GTSAM=$<BOOL:${MSS_WITH_GTSAM}>)

# pipeline: global map export (Estimator::visit_map / export_map) on the box world and corridor.
mss_add_test(test_map_export_component test_map_export_component.cpp)
target_include_directories(test_map_export_component PRIVATE ${CMAKE_SOURCE_DIR}/core/pipeline/src)
target_compile_definitions(test_map_export_component PRIVATE
  MSS_TEST_WITH_GTSAM=$<BOOL:${MSS_WITH_GTSAM}>)

# loop: loop closure end to end (ADR-026): Graph + LoopDetector two-lap drift, Estimator box world.
mss_add_test(test_loop_component test_loop_component.cpp)
target_include_directories(test_loop_component PRIVATE
  ${CMAKE_SOURCE_DIR}/core/pipeline/src ${CMAKE_SOURCE_DIR}/tests/unit)
target_compile_definitions(test_loop_component PRIVATE
  MSS_TEST_WITH_GTSAM=$<BOOL:${MSS_WITH_GTSAM}>)
