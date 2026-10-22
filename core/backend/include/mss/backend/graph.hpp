#pragma once

#include <memory>

#include "mss/backend/config.hpp"
#include "mss/common/result.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"

namespace mss::backend {

/// Factor graph over keyframes, optimised incrementally with iSAM2 (ADR-004).
///
/// The ONLY gateway to GTSAM: this header includes no GTSAM header (pimpl); GTSAM lives in
/// graph.cpp. Every pose quantity crossing this API uses the mss convention: right
/// perturbation, tangent order [rho, phi] (translation first). GTSAM Pose3 uses
/// [phi, rho] (rotation first), so Graph permutes at its boundary, in both directions:
/// vectors `xi_gtsam = P * xi_mss`, covariances `Sigma_gtsam = P * Sigma_mss * P^T`,
/// `P = [0 I; I 0]` (docs/derivations/lie_conventions.md, "Library orders").
/// Variables per keyframe: pose T_w_b, velocity v_w_b, IMU bias (as in NavState).
///
/// Invalid add_*() calls (unknown KeyId, wrong key order, non-finite input, time
/// mismatch beyond the tolerances below) are dropped with one log line per rejected call
/// through the core logger; they never throw and never reach GTSAM.
/// Keyframe boundaries: keyframe time = scan `t_start`. IMU and wheel deltas span the
/// nearest sensor sample times (see imu::Preintegrator, odom::WheelIntegrator), so their
/// end times may differ slightly from the keyframe times.
/// Nothing is optimised until optimize(); factors and keyframes added since the last
/// optimize() are "pending".
///
/// Ownership/threading: single-threaded; owned by pipeline::Estimator and used only on
/// the backend thread (ADR-005). Move-only (owns the iSAM2 instance through `impl_`).
class Graph {
 public:
  /// Inputs:
  ///   - `cfg`: iSAM2 settings and the first-keyframe prior sigmas (prior_*);
  ///   - `imu_cfg`: `acc_random_walk` (m/s^3/sqrt(Hz)) and `gyro_random_walk`
  ///     (rad/s^2/sqrt(Hz)) for the bias random-walk part of add_imu_factor
  ///     (sigma = random_walk * sqrt(dt_s) per axis); `rate_hz` for the IMU time
  ///     tolerance; `gravity_mps2`: the IMU factor uses `g_w = [0, 0, -gravity_mps2]`.
  ///     Other ImuConfig fields are not used;
  ///   - `p_b_g`: GNSS antenna lever arm in b, metres (frames.p_b_g), used by
  ///     add_gnss_factor.
  /// Errors: throws std::invalid_argument when `imu_cfg.rate_hz <= 0`,
  ///   `imu_cfg.gravity_mps2 <= 0`, `keyframe_dist_m <= 0`,
  ///   `keyframe_rot_rad <= 0`, `relinearize_threshold <= 0`, `relinearize_skip <= 0`,
  ///   any prior sigma `<= 0`, a random walk `< 0`, or `p_b_g` non-finite.
  ///   (The keyframe thresholds are applied by the pipeline; Graph validates the whole
  ///   struct it receives.)
  /// Hot path: no. Tests: B-01..B-04 construct it.
  Graph(const BackendConfig& cfg, const imu::ImuConfig& imu_cfg, const common::Vec3& p_b_g);
  ~Graph();
  Graph(const Graph&) = delete;
  Graph& operator=(const Graph&) = delete;
  Graph(Graph&&) noexcept;
  Graph& operator=(Graph&&) noexcept;

  /// Adds the variables (pose, velocity, bias) of a new keyframe at time `t` with `x0`
  /// as the initial guess. Returns its KeyId: 0 for the first keyframe, then +1 per
  /// call. The first keyframe is anchored with priors on pose, velocity and bias
  /// (independent Gaussian priors centred on x0: position prior_pos_sigma_m and rotation
  /// prior_rot_sigma_rad per axis of the [rho, phi] tangent of T_w_b; velocity
  /// prior_vel_sigma_mps; bias prior_bias_acc_sigma_mps2 / prior_bias_gyro_sigma_radps).
  /// Frames: x0.T_w_b, x0.v_w_b in w; x0.bias in b. Units: SI.
  /// Preconditions: `x0.t == t`; `t` strictly greater than the previous keyframe's t.
  ///   A violation is logged; the keyframe is still added (KeyIds stay dense).
  /// Errors: none returned. Hot path: no (per keyframe, about 1 Hz). Tests: B-02, B-04.
  common::KeyId add_keyframe(common::Timestamp t, const common::NavState& x0);

  /// Adds an IMU constraint between keyframes i and j. Implemented as a custom 9x9
  /// preintegration factor built from `delta` (residual on dR, dv, dp with first-order
  /// bias correction through the J_* Jacobians, linearised at delta.bias_lin; gravity
  /// `g_w = [0, 0, -imu_cfg.gravity_mps2]`; delta.cov, block order [phi, p, v], is its
  /// noise, used without permutation) plus a `BetweenFactor<imuBias::ConstantBias>`
  /// between bias_i and bias_j with sigma `random_walk * sqrt(dt_s)`.
  /// NOT gtsam::CombinedImuFactor.
  /// Time check: PreintegratedDelta carries no absolute times, so Graph checks the
  ///   duration: `|round(dt_s * 1e9) - (t_j - t_i)| <= 1.5e9 / imu_cfg.rate_hz` ns (with
  ///   the re-fed boundary sample each end is off by at most half a sample period).
  ///   Beyond that the factor is rejected with one log line.
  /// Frames: delta in b_i (types.hpp). Units: SI.
  /// Preconditions: j == i + 1; both added; delta.dt_s > 0.
  /// Errors: invalid calls dropped and logged (class comment). Hot path: no.
  /// Tests: B-01 (factor Jacobians), B-04; I-03 covers the delta itself.
  void add_imu_factor(common::KeyId i, common::KeyId j, const common::PreintegratedDelta& delta);

  /// Adds a relative-pose (between) factor from LiDAR scan matching:
  /// residual `Log(T_bi_bj^-1 * (T_w_bi^-1 * T_w_bj))` in [rho, phi].
  /// `cov`: 6x6 [rho, phi], right perturbation of T_bi_bj (frame b_j); the pipeline has
  ///   already converted it from l to b and applied degeneracy::relax_covariance when
  ///   degeneracy is enabled. Used as-is (permuted to GTSAM order inside).
  /// `degeneracy`: may be nullptr. When non-null it must be in the same b_j tangent
  ///   (eigvecs rotated with se3_adj(T_b_l) by the pipeline). In this contract it does
  ///   NOT change the factor: it is recorded for diagnostics only (number of degenerate
  ///   keyframes). Any other use (e.g. information projection) needs an ADR.
  /// Frames: T_bi_bj = body at j expressed in body at i. Units: m, rad.
  /// Preconditions: i < j, both added.
  /// Errors: invalid calls dropped and logged. Hot path: no.
  /// Tests: B-01, B-02.
  void add_lidar_factor(common::KeyId i,
                        common::KeyId j,
                        const common::Pose& T_bi_bj,
                        const common::PoseCovariance& cov,
                        const common::DegeneracyReport* degeneracy);

  /// Adds a GNSS position factor on keyframe `k`:
  /// residual `T_e_w * (T_w_b * p_b_g) - m.p_e_g` (3-vector, metres; p_b_g from the
  /// constructor), covariance `m.cov` used as delivered. GnssConfig::sigma_h_m /
  /// sigma_v_m are used by the pipeline only to fill `m.cov` when the fix carries no
  /// covariance (NavSatFix `position_covariance_type == 0`).
  /// `T_e_w` (from gnss::GnssAligner) is a fixed parameter, not a variable.
  /// Graph does not compare m.t with the keyframe time: the pipeline attaches each fix
  /// to the keyframe at (or interpolated to) the fix time.
  /// Frames: m.p_e_g in e; T_e_w maps w to e (`p_e = T_e_w * p_w`). Units: m, m^2.
  /// Preconditions: the fix passed gnss::gate_fix and the chi-square gate (G-02).
  /// Errors: invalid calls dropped and logged. Hot path: no.
  /// Tests: B-01; G-04 (lever arm) end-to-end.
  void add_gnss_factor(common::KeyId k,
                       const common::GnssMeasurement& m,
                       const common::Pose& T_e_w);

  /// Adds a wheel-odometry relative-pose factor between keyframes i and j from
  /// `odom.T_b0_b1` (body frame b, decision 8) and `odom.cov` ([rho, phi], frame b1).
  /// odom.cov is singular in the out-of-plane components (see odom::WheelIntegrator):
  /// the factor constrains only the components with non-zero variance; the
  /// non-holonomic constraint (O-02) is a separate factor designed by the odom/backend
  /// modules.
  /// Time check: `|odom.t0 - t_i| <= 1.5 * P` and `|odom.t1 - t_j| <= 1.5 * P`, with
  ///   P = wheel sample period. Graph never sees WheelSamples, so P is not available to
  ///   it: the pipeline, which observes P (interval between consecutive accepted
  ///   WheelSamples), applies this check before calling and drops the delta with one
  ///   log line beyond it. Graph itself checks only i < j and that both keys exist.
  /// Errors: invalid calls dropped and logged. Hot path: no.
  /// Tests: O-01, O-02, B-01.
  void add_odom_factor(common::KeyId i, common::KeyId j, const common::OdomDelta& odom);

  /// Adds a loop-closure between-factor (lc.i, lc.j, lc.T_bi_bj, lc.cov in [rho, phi],
  /// frame b_j) wrapped in a robust kernel so a wrong loop cannot drag the graph (B-03).
  /// Preconditions: lc.i < lc.j, both added.
  /// Errors: invalid calls dropped and logged. Hot path: no.
  /// Tests: B-02 (square loop), B-03 (outlier loop).
  void add_loop_factor(const common::LoopConstraint& lc);

  /// Runs one iSAM2 update with everything pending and returns the estimate of the
  /// LATEST keyframe: state (T_w_b, v_w_b, bias, t), cov = marginal 6x6 [rho, phi] of its
  /// pose (as marginal_cov), keyframe_id.
  /// Errors (Result::error):
  ///   - "empty graph" when no keyframe was ever added;
  ///   - "optimization failed: <GTSAM message>" when GTSAM throws (e.g. indeterminant
  ///     system). The exception is caught inside. Pending keyframes are kept (with their
  ///     priors and initial values); only the pending factors added since the last
  ///     successful optimize() are dropped. KeyIds remain valid and dense.
  /// Hot path: no (per keyframe; may take tens of ms on the backend thread).
  /// Tests: B-02, B-03, B-04 (iSAM2 vs batch Levenberg-Marquardt < 1e-6).
  common::Result<common::Estimate> optimize();

  /// Marginal covariance of the pose of keyframe `k` as of the last successful
  /// optimize(): 6x6 [rho, phi] (mss order, permuted from GTSAM), right perturbation of
  /// T_w_b(k), frame b. Units: m^2, rad^2 blocks.
  /// Errors: for an unknown or not-yet-optimised `k` returns a matrix filled with quiet
  ///   NaN (never zero, which would claim certainty).
  /// Hot path: no. Tests: B-02, B-04.
  common::PoseCovariance marginal_cov(common::KeyId k) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mss::backend
