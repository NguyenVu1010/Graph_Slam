#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "mss/common/types.hpp"
#include "mss/loop/config.hpp"

namespace mss::loop {

/// Robust loop selection by pairwise consistency maximisation (PCM, Mangelson et al. 2018;
/// ADR-028; docs/derivations/loop_closure.md section 11).
///
/// Holds up to `pcm_max_loops` (<= kMaxLoops) verified loops and the odometry chain of the
/// keyframes. Two loops a, b are consistent when their cycle through the odometry,
///   X = Z_a^-1 * O(i_a -> i_b) * Z_b * O(j_b -> j_a),
/// is the identity within chi2 = Log(X)^T Sigma_X^-1 Log(X) <= pcm_chi2_gate (Sigma_X by
/// right-perturbation propagation of the two loop covariances and the two odometry
/// segments, taken independent). The selected set is a maximum clique of the consistency
/// graph; ties keep the most currently selected loops, then the oldest; a search cut short by
/// pcm_max_clique_steps starts from the current selection (still a clique), so it never makes
/// the selection worse. propose() returns
/// which loops enter and leave the selected set, for the caller to add to or remove from
/// the graph.
///
/// Odometry: O(a -> b) for a < b from prefix sums in O(1): T_0k and the left-tangent
/// covariance S_k = S_{k-1} + Ad(T_0k) C_k Ad(T_0k)^T, so T_ab = T_0a^-1 T_0b and
/// C_ab = Ad(T_0b^-1) (S_b - S_a + sigma_b^2 g g^T) Ad(T_0b^-1)^T, with the yaw-drift term of an
/// unknown constant yaw drift rate (sigma_b = pcm_yaw_drift_deg_per_m, per metre travelled) and
/// g = G_b - G_a, G_k = sum over steps of |p_step| Ad(T_0k) e_yaw (loop_closure.md section 11b);
/// O(b -> a) is its inverse.
///
/// Ownership/threading: single-threaded, owned by the pipeline's EstimatorCore and called at
/// keyframe time (ADR-028). No GTSAM.
class LoopSelector {
 public:
  static constexpr std::size_t kMaxLoops = 64;  // one bit per loop in a 64-bit mask

  /// What propose() did.
  struct Decision {
    bool selected = false;  ///< the proposed loop is in the selected set
    int n_consistent = 0;   ///< loops held that are consistent with the proposed one
    int clique_size = 0;    ///< size of the selected set after the call
    bool frozen = false;    ///< the oldest selected loop left PCM to make room (stays in graph)
  };

  /// When `cfg.enabled && cfg.pcm_enabled`: validates pcm_chi2_gate (> 0), pcm_max_loops
  /// ([1, kMaxLoops]), pcm_odometry_sigma_scale (>= 1), pcm_yaw_drift_rad_per_m (>= 0) and
  /// pcm_max_clique_steps (> 0) and
  /// reserves the odometry chain for
  /// `reserve_keyframes` keyframes (it grows past that, per keyframe) and the loop lists.
  /// Errors: throws std::invalid_argument on an invalid value. Hot path: no.
  LoopSelector(const LoopConfig& cfg, std::size_t reserve_keyframes);

  /// Appends the odometry increment ending at keyframe k: T_{k-1,k} (body k-1 to body k) and
  /// its covariance ([rho, phi], right perturbation, frame b_k), held scaled by
  /// pcm_odometry_sigma_scale^2. Keyframe 0 needs no call.
  /// Returns false (and ignores the call) unless k is the next keyframe (k == chain size)
  /// and the inputs are finite. Allocation: amortised, per keyframe (not per point).
  bool add_odometry(common::KeyId k,
                    const common::Pose& T_prev_k,
                    const common::PoseCovariance& cov);

  /// Takes a verified loop (lc.i < lc.j, both in the odometry chain, finite, covariance
  /// positive definite; else rejected: selected = false, nothing changes), tests it against
  /// every held loop and re-selects. Afterwards to_add() lists the loops that entered the
  /// selected set (the proposed one included when selected) and to_remove() those that left
  /// it. No allocation (lists reserved in the constructor).
  Decision propose(const common::LoopConstraint& lc);

  const std::vector<common::LoopConstraint>& to_add() const { return to_add_; }
  const std::vector<std::pair<common::KeyId, common::KeyId>>& to_remove() const {
    return to_remove_;
  }

  /// Drops the held loop (i, j) (the most recent such) from PCM, selected or not, without
  /// reporting it in to_remove(): for the caller to re-sync after the graph lost the loop (a
  /// failed update dropped it). Returns false when no such loop is held. Review H3 m-1.
  bool forget(common::KeyId i, common::KeyId j);

  /// Odometry relative pose and covariance from keyframe a to keyframe b (either order).
  /// False when a or b is outside the chain.
  bool odometry(common::KeyId a,
                common::KeyId b,
                common::Pose& T_ab,
                common::PoseCovariance& cov) const;

  /// chi2 of the cycle of loops a and b (the pairwise test statistic); +inf when it cannot
  /// be formed (outside the chain, covariance not positive definite).
  double pair_chi2(const common::LoopConstraint& a, const common::LoopConstraint& b) const;

  std::size_t n_loops() const;  ///< loops held
  std::size_t n_selected() const;

 private:
  void select();
  void clique(std::uint64_t r, std::uint64_t p, std::uint64_t x);

  LoopConfig cfg_;
  std::vector<common::Pose> T_0k_;           // odometry prefix poses, index = KeyId
  std::vector<common::PoseCovariance> S_k_;  // left-tangent prefix covariance sums
  std::vector<common::Vec6> G_k_;            // left-tangent prefix yaw-drift derivatives
  std::array<common::LoopConstraint, kMaxLoops> loops_{};
  std::array<std::uint64_t, kMaxLoops> age_{};  // insertion counter per slot (older = smaller)
  std::array<std::uint64_t, kMaxLoops> adj_{};  // consistency bitsets
  std::uint64_t used_ = 0;                      // occupied slots
  std::uint64_t selected_ = 0;                  // selected slots
  std::uint64_t counter_ = 0;
  // Max-clique search state.
  std::uint64_t best_ = 0;
  long steps_ = 0;
  std::vector<common::LoopConstraint> to_add_;
  std::vector<std::pair<common::KeyId, common::KeyId>> to_remove_;
};

}  // namespace mss::loop
