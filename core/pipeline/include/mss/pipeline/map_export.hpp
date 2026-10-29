#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "mss/common/span.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"

namespace mss::pipeline {

/// One stored keyframe of the global map, as Estimator::visit_map() hands it out (ADR-016
/// decision 6 and its 2026-10-29 amendment).
struct MapKeyframe {
  common::KeyId id = 0;
  common::Timestamp t{};  ///< keyframe time (scan t_start)
  /// T_w_l = T_w_b(k) * T_b_l with T_w_b(k) the backend's optimised pose at the time of the
  /// call (backend::Graph::keyframe_state), never the pose used at insertion.
  common::Pose T_w_l;
  /// The keyframe cloud in l as stored (deskewed to `t`, downsampled by
  /// preprocess.downsample_voxel_m then map.keyframe_voxel_m): x0 y0 z0 x1 ..., m.
  /// Valid only during the visitor call.
  common::Span<const float> xyz_l;
};

/// Visitor of Estimator::visit_map(), called once per exported keyframe in id order.
using MapKeyframeVisitor = std::function<void(const MapKeyframe&)>;

/// Summary of one map export.
struct MapExportInfo {
  std::size_t n_keyframes = 0;   ///< keyframes exported (store entries with an optimised pose)
  std::size_t n_no_pose = 0;     ///< store entries skipped: the backend has no estimate for them
  std::uint64_t n_evicted = 0;   ///< keyframes evicted from the store (map.max_keyframes) so far;
                                 ///< their points are not in the map
  std::size_t n_points_in = 0;   ///< points of the exported keyframes
  std::size_t n_points_out = 0;  ///< points written by export_map() (after map_export.voxel_m);
                                 ///< equal to n_points_in for visit_map()
};

}  // namespace mss::pipeline
