#pragma once

#include <cstddef>

namespace mss::map {

// YAML: map.*  Loaded and validated by mss::pipeline::load_config.
struct MapConfig {
  double voxel_m = 0.0;          // m, voxel edge (> 0)
  int max_points_per_voxel = 0;  // points kept per voxel (> 0)
  int knn_k = 0;                 // neighbours used for a plane fit (> 0)
  double prune_radius_m = 0.0;   // m, voxels farther than this from the body are removed
  double prune_period_s = 0.0;   // s, pruning period
  std::size_t max_voxels = 0;    // hard cap on voxel count (> 0)
};

}  // namespace mss::map
