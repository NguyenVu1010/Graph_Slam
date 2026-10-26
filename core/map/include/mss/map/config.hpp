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
  // ADR-016: keyframe store and rebuildable cache (consumed by pipeline, not by VoxelMap).
  double keyframe_voxel_m =
      0.0;  // m, extra downsampling of the keyframe cloud kept in the store (> 0)
  std::size_t max_keyframes = 0;  // keyframe store bound; oldest dropped from the store (> 0)
  int window_keyframes = 0;       // keyframes in the VoxelMap cache (> 0)
  double rebuild_trans_m =
      0.0;  // m, pose change of the oldest window keyframe that triggers a rebuild (> 0)
  double rebuild_rot_rad = 0.0;  // rad (YAML key rebuild_rot_deg), same trigger for rotation (> 0)
};

}  // namespace mss::map
