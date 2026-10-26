#pragma once

namespace mss::preprocess {

// YAML: preprocess.*  Loaded and validated by mss::pipeline::load_config.
struct PreprocessConfig {
  double min_range_m = 0.0;         // m, drop returns closer than this
  double max_range_m = 0.0;         // m, drop returns farther than this
  double downsample_voxel_m = 0.0;  // m, voxel-grid edge for scan downsampling
  bool deskew_enabled = false;      // undistort points to t_start with the IMU trajectory
};

}  // namespace mss::preprocess
