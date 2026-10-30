#pragma once

#include <vector>

#include <Eigen/Core>

namespace mss::pipeline::detail {

/// Voxel-grid centroid downsampling of the exported map (map_export.voxel_m).
/// Replaces `points` by one point per occupied voxel `floor(p / voxel_m)`: the mean of the
/// points in it (accumulated in double), in the order in which the voxels are first met, so
/// the result is deterministic for a given input order. `voxel_m` that is 0, negative, NaN or
/// below 1 mm leaves `points`
/// unchanged. When downsampling, a point that is not finite or whose voxel index exceeds 4e18
/// is dropped (never an out-of-range int64 cast). Points keep their frame (w for the map).
/// Allocates (hash map, output); off the
/// hot path. Internal to the pipeline.
void voxel_centroids(std::vector<Eigen::Vector3f>& points, double voxel_m);

}  // namespace mss::pipeline::detail
