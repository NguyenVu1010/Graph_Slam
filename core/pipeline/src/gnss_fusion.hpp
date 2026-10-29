#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/StdVector>

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

/// A pre-alignment fix released as a factor on an earlier or the current keyframe (ADR-022):
/// `m` is already shifted to the time `t_k` of keyframe `kf`.
struct EarlyFactor {
  common::GnssMeasurement m;
  common::KeyId kf = 0;
  common::Timestamp t_k{};
};

/// GNSS fusion front: pre-gate, chi-square innovation gate (G-02), w->e alignment (GnssAligner,
/// or GnssTrackYawAligner with gnss.align_yaw_from_track, ADR-023) and the
/// fixes waiting for the next keyframe (docs/derivations/pipeline.md section 5).
/// With `gnss.attach_prealign_fixes` (ADR-022) it also keeps the fixes received before the
/// first alignment in a bounded buffer, assigns each to its nearest keyframe in time and
/// releases them as factors once aligned.
/// Internal to the pipeline. Single-threaded: owned by EstimatorCore. No allocation after
/// construction (fixed rings, a buffer reserved once, literal error reasons).
class GnssFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// `cfg`: max_cov_m2, chi2_gate, the aligner thresholds and the pre-alignment buffer
  /// (attach_prealign_fixes; prealign_max_fixes entries reserved here when on); `p_b_g`
  /// lever arm in b (m); `T_b_o` the odometry frame (frames.T_b_o), whose forward speed the
  /// wheel odometer measures (track-yaw wheel path, ADR-023 design 2b). Throws std::bad_alloc.
  GnssFusion(const gnss::GnssConfig& cfg,
             const common::Vec3& p_b_g,
             const common::Pose& T_b_o = common::Pose{});

  /// Handles one fix. `T_w_b_fix`: front-end body pose at `m.t`; `P_w_b`: its 6x6
  /// [rho, phi] covariance (treated as zero when any entry is non-finite).
  /// Order: gnss::gate_fix (its reason is returned); once aligned, the chi-square gate
  /// (its reason, e.g. "chi-square gate exceeded", is returned and the aligner is not
  /// fed); then the aligner update (refreshes T_e_w() on success). When aligned after
  /// that the fix is queued for collect() (full queue: oldest dropped and counted) and ok
  /// is returned, else Status::error("gnss not aligned yet").
  /// ADR-022 (attach_prealign_fixes): before the first alignment, a not-aligned fix is also
  /// kept in the pre-alignment buffer with `T_w_b_fix` and `P_w_b` (buffer full: the new fix
  /// is not kept and is counted in n_prealign_dropped(), so the earlier static ones stay).
  /// After the first alignment nothing more is buffered, also not after a realign.
  /// Other errors: "non-finite pose" (T_w_b_fix not finite).
  /// `outcome` (optional) receives the same decision as an enum. All reasons are
  /// literals: no allocation unless the caller calls Status::reason().
  /// `wheel_odometer_m`: absolute signed wheel odometer reading at the fix (m), NaN without
  /// wheel data; used only by the track-yaw aligner (ADR-023) for the direction of travel.
  common::Status on_fix(const common::GnssMeasurement& m,
                        const common::Pose& T_w_b_fix,
                        const common::PoseCovariance& P_w_b,
                        FixOutcome* outcome = nullptr,
                        double wheel_odometer_m = std::numeric_limits<double>::quiet_NaN());

  /// Moves every queued fix with `t <= t_k` into `out[0, n)`, shifted to the keyframe
  /// pose `T_w_bk` (pipeline.md section 5), and returns n. Later fixes stay queued.
  std::size_t collect(common::Timestamp t_k,
                      const common::Pose& T_w_bk,
                      std::array<common::GnssMeasurement, kMaxAuxQueue>& out);

  /// ADR-022: a fix received before keyframe 0 exists (no front-end pose yet). It must pass
  /// gnss::gate_fix (its literal reason is returned, nothing kept) and is kept until
  /// on_first_keyframe(); when the buffer is full the oldest buffered fix is dropped and
  /// counted in n_prealign_dropped(). Ignored (ok) unless attach_prealign_fixes and before
  /// the first alignment. No allocation.
  common::Status on_preinit_fix(const common::GnssMeasurement& m);

  /// ADR-022: keyframe 0 (`id` at `t_k`, pose `T_w_b0`, covariance `P0`) was added. When
  /// `static_ok` (keyframe 0 is the IMU initialisation scan, so the body was at rest over the
  /// initialisation window) the fixes from on_preinit_fix() with `static_from <= t <= t_k`
  /// are assigned to it with no shift (pose `T_w_b0`); the others are dropped and counted in
  /// n_prealign_not_static().
  /// Records keyframe 0 as the previous keyframe of on_keyframe().
  void on_first_keyframe(common::KeyId id,
                         common::Timestamp t_k,
                         const common::Pose& T_w_b0,
                         const common::PoseCovariance& P0,
                         common::Timestamp static_from,
                         bool static_ok);

  /// ADR-022: keyframe `id` at `t_k`, front-end pose `T_w_bk` (the pose given to collect()),
  /// is being added. Each buffered fix with t <= t_k not yet assigned goes to the nearer in
  /// time of the previous keyframe (set_keyframe_reference()) and this one; a tie goes to the
  /// previous one. Its shift is the front-end antenna displacement from the fix to that
  /// keyframe: T_w_ref p_b_g - T_w_b_fix p_b_g (previous keyframe, its reference pose) or
  /// T_w_bk p_b_g - T_w_b_fix p_b_g (this one, as collect()). T_w_b_fix was predicted from the
  /// front-end state of the last scan before the fix, so a backward shift also carries the
  /// scan-match chain from T_w_ref to that scan. With set_shift_oracle() both shifts come from
  /// the oracle (test-only).
  void on_keyframe(common::KeyId id, common::Timestamp t_k, const common::Pose& T_w_bk);

  /// ADR-022: the reference pose of keyframe `id` after its optimisation (the state later
  /// front-end predictions start from); the target of backward shifts in on_keyframe().
  void set_keyframe_reference(common::KeyId id, common::Timestamp t_k, const common::Pose& T_w_ref);

  /// ADR-022: once aligned(), moves up to out.size() assigned buffered fixes into `out`, each
  /// gated by the chi-square gate at its fix time with the current T_e_w() (rejected ones are
  /// removed and counted in n_prealign_rejected()) and shifted to its keyframe:
  /// p_e_g' = p_e_g + R_e_w shift_w, cov unchanged. Returns the number written; call until it
  /// returns 0. Each buffered fix is released at most once. Not aligned: returns 0.
  std::size_t take_prealign(std::array<EarlyFactor, kMaxAuxQueue>& out);

  [[nodiscard]] bool aligned() const { return aligned_; }
  /// Test-only (CoreTestHooks): use `T_e_w` from now on, aligned, without the aligner.
  void force_T_e_w(const common::Pose& T_e_w) {
    T_e_w_ = T_e_w;
    aligned_ = true;
    forced_ = true;
  }
  /// Test-only (CoreTestHooks::gnss_shift_oracle): collect() takes the antenna displacement
  /// in w from the fix time to the keyframe time from `oracle(t_fix, t_k)` instead of the
  /// front-end poses; so does on_keyframe() for the pre-alignment fixes. `oracle` must outlive this
  /// object; nullptr restores the default.
  /// Test-only (CoreTestHooks::gnss_first_align_yaw_offset_rad): for the first
  /// gnss.align_yaw_chunk_fixes fixes from the first alignment on (until the aligner re-estimates
  /// its yaw), the published T_e_w (the centre of E's prior, the gate, the shift) is turned by
  /// `rad` about the antenna position of each fix; later fixes use the aligner's estimate.
  void set_first_alignment_yaw_offset(double rad) { first_align_yaw_offset_rad_ = rad; }
  void set_shift_oracle(
      const std::function<common::Vec3(common::Timestamp, common::Timestamp)>* oracle) {
    shift_oracle_ = oracle;
  }
  /// Times the fusion went from not aligned to aligned (first alignment and each realign).
  [[nodiscard]] std::uint64_t n_alignments() const { return n_alignments_; }
  /// ADR-023: predicted yaw 1-sigma (rad) of the track-yaw aligner at its last chunk boundary;
  /// NaN when gnss.align_yaw_from_track is false (GnssAligner reports none).
  [[nodiscard]] double align_yaw_sigma_rad() const {
    return cfg_.align_yaw_from_track ? track_aligner_.yaw_sigma_rad()
                                     : std::numeric_limits<double>::quiet_NaN();
  }
  /// ADR-023: the track-yaw aligner in use (its counters are meaningful only when
  /// gnss.align_yaw_from_track is true).
  [[nodiscard]] const gnss::GnssTrackYawAligner& track_aligner() const { return track_aligner_; }
  /// Latest aligner estimate (identity until aligned()). Maps w to e.
  [[nodiscard]] const common::Pose& T_e_w() const { return T_e_w_; }
  /// Accepted fixes dropped from the full pending queue so far.
  [[nodiscard]] std::uint64_t n_pending_dropped() const { return n_pending_dropped_; }
  /// Aligner restarts so far: after `max_consecutive_rejects` chi-square rejections in a row
  /// the aligner is rebuilt empty (no allocation), aligned() becomes false and the rejected
  /// fix is the first pair of the new alignment (pipeline.md section 5).
  [[nodiscard]] std::uint64_t n_realign() const { return n_realign_; }
  /// ADR-022 counters: pre-alignment fixes released as factors (of those: static fixes on
  /// keyframe 0); dropped because the buffer was full; received before keyframe 0 but not
  /// in its static window, or keyframe 0 not at rest; rejected by the chi-square gate at
  /// release.
  [[nodiscard]] std::uint64_t n_prealign_attached() const { return n_prealign_attached_; }
  [[nodiscard]] std::uint64_t n_prealign_static() const { return n_prealign_static_; }
  [[nodiscard]] std::uint64_t n_prealign_dropped() const { return n_prealign_dropped_; }
  [[nodiscard]] std::uint64_t n_prealign_not_static() const { return n_prealign_not_static_; }
  [[nodiscard]] std::uint64_t n_prealign_rejected() const { return n_prealign_rejected_; }
  /// Fixes now in the pre-alignment buffer.
  [[nodiscard]] std::size_t n_prealign_buffered() const { return early_.size(); }

 private:
  struct PendingFix {
    common::GnssMeasurement m;
    common::Pose T_w_b_fix;
  };
  // A fix received before the first alignment (ADR-022).
  struct EarlyFix {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    common::GnssMeasurement m;
    common::Pose T_w_b_fix;        // front-end pose at m.t (keyframe 0's for a static fix)
    common::PoseCovariance P_w_b;  // its [rho, phi] covariance (non-finite: treated as zero)
    common::Vec3 shift_w;          // m, antenna displacement in w from m.t to the keyframe
    common::KeyId kf = 0;          // assigned keyframe
    common::Timestamp t_k{};       // its time
    bool preinit = false;          // received before keyframe 0 (no pose yet)
    bool assigned = false;
    bool static_kf0 = false;  // a static-window fix on keyframe 0
  };
  // G-02 chi-square gate of `m` at body pose `T_w_b` with covariance `P_w_b` (P := 0 when
  // any entry is non-finite) against T_e_w_.
  [[nodiscard]] common::Status chi_square(const common::GnssMeasurement& m,
                                          const common::Pose& T_w_b,
                                          const common::PoseCovariance& P_w_b) const;

  gnss::GnssConfig cfg_;  // kept to rebuild the aligner
  double max_cov_m2_;
  double chi2_gate_;
  int consecutive_rejects_ = 0;
  bool forced_ = false;  // force_T_e_w: the aligner is never run
  std::uint64_t n_realign_ = 0;
  common::Vec3 p_b_g_;
  common::Pose T_b_o_;                       // odometry frame in b, for the track-yaw aligner
  double first_align_yaw_offset_rad_ = 0.0;  // test-only, see set_first_alignment_yaw_offset
  int offset_fixes_left_ = 0;                // test-only
  gnss::GnssAligner aligner_;
  gnss::GnssTrackYawAligner track_aligner_;  // ADR-023, used when cfg_.align_yaw_from_track
  std::uint64_t n_alignments_ = 0;
  bool aligned_ = false;
  common::Pose T_e_w_;
  BoundedQueue<PendingFix, kMaxAuxQueue> pending_;
  std::uint64_t n_pending_dropped_ = 0;
  const std::function<common::Vec3(common::Timestamp, common::Timestamp)>* shift_oracle_ =
      nullptr;  // test-only
  // ADR-022 pre-alignment buffer, in arrival order: capacity cfg_.prealign_max_fixes reserved
  // at construction when attach_prealign_fixes, never grown past it (no allocation after).
  std::vector<EarlyFix, Eigen::aligned_allocator<EarlyFix>> early_;
  bool early_open_ = false;  // buffering: attach_prealign_fixes and never aligned yet
  bool have_prev_kf_ = false;
  common::KeyId prev_kf_ = 0;
  common::Timestamp prev_kf_t_{};
  common::Pose prev_kf_T_;
  std::uint64_t n_prealign_attached_ = 0;
  std::uint64_t n_prealign_static_ = 0;
  std::uint64_t n_prealign_dropped_ = 0;
  std::uint64_t n_prealign_not_static_ = 0;
  std::uint64_t n_prealign_rejected_ = 0;
};

}  // namespace mss::pipeline::detail
