#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "mss/backend/config.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"

namespace mss::pipeline::detail {

/// New keyframe when the body moved `>= keyframe_dist_m` or rotated `>= keyframe_rot_rad`
/// from the reference pose of the last keyframe (docs/derivations/pipeline.md section 6).
bool is_new_keyframe(const common::Pose& T_w_b_ref,
                     const common::Pose& T_w_b,
                     const backend::BackendConfig& cfg);

/// True when `b` differs from `a` by more than `trans_m` (m) or `rot_rad` (rad), strict
/// (map rebuild trigger, ADR-016, pipeline.md section 6).
bool pose_moved(const common::Pose& a, const common::Pose& b, double trans_m, double rot_rad);

/// Wheel delta time check (graph.hpp add_odom_factor): `2 |t0 - t_i| <= 3 P` and
/// `2 |t1 - t_j| <= 3 P` in integer ns, `P = period_ns` the observed wheel interval.
/// False when `period_ns <= 0`.
bool odom_delta_in_time(const common::OdomDelta& d,
                        common::Timestamp t_i,
                        common::Timestamp t_j,
                        std::int64_t period_ns);

/// One keyframe of the VoxelMap cache and the body pose its cloud was inserted with.
struct WindowEntry {
  common::KeyId id = 0;
  common::Pose T_w_b_inserted;
};

/// The last `window_keyframes` keyframes held in the VoxelMap cache (ADR-016), as a ring
/// allocated once. Internal to the pipeline.
class MapWindow {
 public:
  /// Throws std::invalid_argument when `window_keyframes == 0`.
  explicit MapWindow(std::size_t window_keyframes);
  /// Appends; when full the oldest entry is dropped.
  void push(common::KeyId id, const common::Pose& T_w_b);
  [[nodiscard]] std::size_t size() const { return size_; }
  /// Entry i, 0 = oldest. Precondition: i < size().
  [[nodiscard]] const WindowEntry& at(std::size_t i) const {
    return slots_[(head_ + i) % slots_.size()];
  }
  void set_pose(std::size_t i, const common::Pose& T_w_b);

 private:
  std::vector<WindowEntry> slots_;
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};

}  // namespace mss::pipeline::detail
