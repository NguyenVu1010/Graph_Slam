#pragma once

#include <cstdint>
#include <vector>

#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"

namespace mss::pipeline {

/// One keyframe of the backend graph, as Estimator::graph_snapshot() hands it out (ADR-031).
struct GraphKeyframe {
  common::KeyId id = 0;
  common::Timestamp t{};  ///< keyframe time (scan t_start)
  /// T_w_b(k): the backend's optimised body pose at the time of the call
  /// (backend::Graph::keyframe_state), the pose kf_opt.tum and visit_map() use.
  common::Pose T_w_b;
};

/// State of a loop that entered the graph (ADR-026, ADR-028, ADR-030).
enum class LoopState : std::uint8_t {
  kActive,    ///< in the graph with the Cauchy kernel (or without PCM)
  kReleased,  ///< in the graph without the kernel (ADR-030 loop_closure.kernel_release)
  kRemoved,   ///< entered the graph, later removed by PCM (ADR-028); not in the graph now
};

/// One loop between keyframes i (older, the candidate) and j (the query).
struct GraphLoop {
  common::KeyId i = 0;
  common::KeyId j = 0;
  common::Timestamp t_i{};  ///< time of keyframe i ({} when the backend has no estimate for it)
  common::Timestamp t_j{};  ///< time of keyframe j (likewise)
  LoopState state = LoopState::kActive;
};

/// The keyframes and loops of the backend graph (Estimator::graph_snapshot()). In the no-GTSAM
/// stub build the backend has no estimates, so a snapshot is always empty.
struct GraphSnapshot {
  /// Every keyframe the backend has an estimate for, in id order, including keyframes evicted
  /// from the map store (map.max_keyframes): the whole trajectory.
  std::vector<GraphKeyframe> keyframes;
  /// Every loop in the graph now (kActive, kReleased), in the order they entered, then every loop
  /// PCM removed and that did not come back (kRemoved), in removal order. A loop dropped with a
  /// failed update never entered the graph and is not listed.
  std::vector<GraphLoop> loops;
};

}  // namespace mss::pipeline
