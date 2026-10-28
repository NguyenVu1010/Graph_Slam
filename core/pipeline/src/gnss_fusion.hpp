#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "mss/common/result.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/gnss/config.hpp"
#include "mss/gnss/gnss.hpp"
#include "mss/pipeline/bounded_queue.hpp"

namespace mss::pipeline::detail {

/// What GnssFusion::on_fix did with a fix, so callers can branch without building the
/// Status reason string (Status::reason() may allocate).
enum class FixOutcome : std::uint8_t {
  kAccepted,      ///< queued for the next keyframe
  kNotAligned,    ///< fed to the aligner, no T_e_w yet
  kGateFix,       ///< rejected by gnss::gate_fix
  kChiSquare,     ///< rejected by the chi-square gate (or a degenerate innovation)
  kNonFinitePose  ///< T_w_b_fix not finite
};

/// GNSS fusion front: pre-gate, chi-square innovation gate (G-02), w->e alignment and the
/// fixes waiting for the next keyframe (docs/derivations/pipeline.md section 5).
/// Internal to the pipeline. Single-threaded: owned by EstimatorCore. No allocation after
/// construction (fixed rings, literal error reasons).
class GnssFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// `cfg`: max_cov_m2, chi2_gate and the aligner thresholds; `p_b_g` lever arm in b (m).
  GnssFusion(const gnss::GnssConfig& cfg, const common::Vec3& p_b_g);

  /// Handles one fix. `T_w_b_fix`: front-end body pose at `m.t`; `P_w_b`: its 6x6
  /// [rho, phi] covariance (treated as zero when any entry is non-finite).
  /// Order: gnss::gate_fix (its reason is returned); once aligned, the chi-square gate
  /// (its reason, e.g. "chi-square gate exceeded", is returned and the aligner is not
  /// fed); then the aligner update (refreshes T_e_w() on success). When aligned after
  /// that the fix is queued for collect() (full queue: oldest dropped and counted) and ok
  /// is returned, else Status::error("gnss not aligned yet").
  /// Other errors: "non-finite pose" (T_w_b_fix not finite).
  /// `outcome` (optional) receives the same decision as an enum. All reasons are
  /// literals: no allocation unless the caller calls Status::reason().
  common::Status on_fix(const common::GnssMeasurement& m,
                        const common::Pose& T_w_b_fix,
                        const common::PoseCovariance& P_w_b,
                        FixOutcome* outcome = nullptr);

  /// Moves every queued fix with `t <= t_k` into `out[0, n)`, shifted to the keyframe
  /// pose `T_w_bk` (pipeline.md section 5), and returns n. Later fixes stay queued.
  std::size_t collect(common::Timestamp t_k,
                      const common::Pose& T_w_bk,
                      std::array<common::GnssMeasurement, kMaxAuxQueue>& out);

  [[nodiscard]] bool aligned() const { return aligned_; }
  /// Test-only (CoreTestHooks): use `T_e_w` from now on, aligned, without the aligner.
  void force_T_e_w(const common::Pose& T_e_w) {
    T_e_w_ = T_e_w;
    aligned_ = true;
    forced_ = true;
  }
  /// Test-only (CoreTestHooks::gnss_shift_oracle): collect() takes the antenna displacement
  /// in w from the fix time to the keyframe time from `oracle(t_fix, t_k)` instead of the
  /// front-end poses. `oracle` must outlive this object; nullptr restores the default.
  void set_shift_oracle(
      const std::function<common::Vec3(common::Timestamp, common::Timestamp)>* oracle) {
    shift_oracle_ = oracle;
  }
  /// Latest aligner estimate (identity until aligned()). Maps w to e.
  [[nodiscard]] const common::Pose& T_e_w() const { return T_e_w_; }
  /// Accepted fixes dropped from the full pending queue so far.
  [[nodiscard]] std::uint64_t n_pending_dropped() const { return n_pending_dropped_; }
  /// Aligner restarts so far: after `max_consecutive_rejects` chi-square rejections in a row
  /// the aligner is rebuilt empty (no allocation), aligned() becomes false and the rejected
  /// fix is the first pair of the new alignment (pipeline.md section 5).
  [[nodiscard]] std::uint64_t n_realign() const { return n_realign_; }

 private:
  struct PendingFix {
    common::GnssMeasurement m;
    common::Pose T_w_b_fix;
  };

  gnss::GnssConfig cfg_;  // kept to rebuild the aligner
  double max_cov_m2_;
  double chi2_gate_;
  int consecutive_rejects_ = 0;
  bool forced_ = false;  // force_T_e_w: the aligner is never run
  std::uint64_t n_realign_ = 0;
  common::Vec3 p_b_g_;
  gnss::GnssAligner aligner_;
  bool aligned_ = false;
  common::Pose T_e_w_;
  BoundedQueue<PendingFix, kMaxAuxQueue> pending_;
  std::uint64_t n_pending_dropped_ = 0;
  const std::function<common::Vec3(common::Timestamp, common::Timestamp)>* shift_oracle_ =
      nullptr;  // test-only
};

}  // namespace mss::pipeline::detail
