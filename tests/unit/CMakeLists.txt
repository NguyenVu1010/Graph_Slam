mss_add_test(test_common test_timestamp.cpp test_result.cpp test_pose.cpp alloc_counter_impl.cpp)
mss_add_test(test_lie test_lie.cpp)
mss_add_test(test_scan_frame test_scan_frame.cpp alloc_counter_impl.cpp)
mss_add_test(test_types test_types.cpp)
mss_add_test(test_config test_config.cpp)
mss_add_test(test_interfaces_compile test_interfaces_compile.cpp)
mss_add_test(test_dataset_io test_dataset_io.cpp test_pcd_writer.cpp alloc_counter_impl.cpp)
mss_add_test(test_estimator test_estimator.cpp alloc_counter_impl.cpp)
mss_add_test(test_degeneracy test_degeneracy.cpp alloc_counter_impl.cpp)
mss_add_test(test_voxel_map test_voxel_map.cpp alloc_counter_impl.cpp)
add_subdirectory(odom)
mss_add_test(test_preprocess test_preprocessor.cpp test_preprocess_deskew_table.cpp
  test_preprocess_voxel_set.cpp alloc_counter_impl.cpp)
mss_add_test(test_imu test_imu_preintegrator.cpp test_imu_covariance.cpp test_imu_initializer.cpp
             alloc_counter_impl.cpp)
mss_add_test(test_gnss test_gnss_geodesy.cpp test_gnss_gate.cpp test_gnss_aligner.cpp
  test_gnss_track_yaw.cpp test_gnss_track_yaw_2b.cpp test_gnss_track_yaw_study.cpp
  alloc_counter_impl.cpp)
target_include_directories(test_gnss PRIVATE ${CMAKE_SOURCE_DIR}/core/gnss/src)
mss_add_test(test_scan_match test_scan_match.cpp)
target_include_directories(test_scan_match PRIVATE ${CMAKE_SOURCE_DIR}/core/scan_match/src)
# backend API (both builds) and the no-GTSAM stub.
mss_add_test(test_backend_api test_backend_api.cpp)
if(NOT MSS_WITH_GTSAM)
  mss_add_test(test_backend_stub test_backend_stub.cpp)
endif()
# pipeline internals (core/pipeline/src helpers, not installed).
mss_add_test(test_pipeline_internals test_pipeline_internals.cpp test_pipeline_gnss_fusion.cpp
  test_map_downsample.cpp
  alloc_counter_impl.cpp)
target_include_directories(test_pipeline_internals PRIVATE ${CMAKE_SOURCE_DIR}/core/pipeline/src)
