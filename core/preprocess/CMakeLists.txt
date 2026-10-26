# preprocess: range filter, IMU deskew and voxel downsampling of one LiDAR scan.
add_library(mss_preprocess STATIC src/preprocessor.cpp src/deskew_table.cpp src/voxel_set.cpp)
add_library(mss::preprocess ALIAS mss_preprocess)
mss_set_target_defaults(mss_preprocess)
target_include_directories(mss_preprocess PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_preprocess PUBLIC mss::common mss::imu)  # ImuTrajectory
