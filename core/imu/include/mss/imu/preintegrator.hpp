#pragma once

#include "mss/common/result.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"

namespace mss::imu {

/// On-manifold IMU preintegration between two keyframes (Forster et al., GTSAM scheme).
/// Agreement is with gtsam::ManifoldPreintegration; PreintegratedCombinedMeasurements differs
/// under GTSAM_TANGENT_PREINTEGRATION (docs/derivations/imu_preintegration.md §6).
///
/// Accumulates `PreintegratedDelta` (types.hpp): dR = R_bi_bj, dv and dp in b_i, dt_s,
/// the 9x9 covariance in GTSAM block order [phi (rot), p, v] (decision 5: no permutation
/// needed against GTSAM), and the bias Jacobians J_dR_bg, J_dv_ba, J_dv_bg, J_dp_ba,
/// J_dp_bg for first-order bias correction. Gravity is NOT in the deltas; predict() adds it.
/// Integration rule (Euler, as GTSAM integrateMeasurement): the first sample after reset()
/// only sets the start time `t_i` and integrates nothing; each following sample k uses
/// ITS OWN measurement (minus `bias_lin`) held over `dt = seconds_between(t_{k-1}, t_k)`.
/// So the delta covers [t of first sample, t of last sample].
/// Keyframe boundaries (pipeline contract): keyframe time = scan `t_start`. After
/// reset() the pipeline re-feeds the last IMU sample of the previous interval as the
/// first sample of the new one, so consecutive deltas chain without gaps. Deltas
/// therefore span the nearest IMU sample times, not the exact keyframe times;
/// backend::Graph::add_imu_factor accepts the delta when
/// `|round(dt_s * 1e9) - (t_j - t_i)| <= 1.5e9 / rate_hz` ns and rejects (logs) the
/// factor beyond that.
/// Noise: continuous-time densities from ImuConfig (acc_noise_density,
/// gyro_noise_density), discretised as density^2 / dt; integration covariance zero.
/// Bias random walk is not part of this 9x9 covariance (it belongs to the backend's bias
/// factor).
///
/// Ownership/threading: single-threaded; owned by pipeline::Estimator and used only on the
/// front-end thread (the pipeline may hold several instances, e.g. keyframe delta and
/// per-scan prediction).
/// Fixed-size storage, no heap memory.
class Preintegrator {
 public:
  /// Validates `cfg`; starts in the reset state with zero bias.
  /// Errors: throws std::invalid_argument when a noise density is negative or
  ///   `rate_hz <= 0`.
  /// Hot path: no. Tests: I-01..I-05 construct it.
  explicit Preintegrator(const ImuConfig& cfg);

  /// Clears the delta (dR = I, dv = dp = 0, dt_s = 0, cov = 0, Jacobians = 0), sets
  /// `delta().bias_lin = bias`, and forgets the previous sample time, so the next
  /// integrate() call is a "first sample".
  /// Frames: bias in b. Units: rad/s (bg), m/s^2 (ba).
  /// Errors: none. Hot path: yes (per keyframe); no allocation, logging or exceptions.
  /// Tests: I-01..I-05.
  void reset(const common::ImuBias& bias);

  /// Integrates one sample (see the class comment for the rule).
  /// Frames: `s.acc_b` (specific force), `s.gyro_b` in b. Units: m/s^2, rad/s, ns.
  /// Preconditions: none checked beyond the errors below.
  /// Errors: returns Status::error and leaves the delta unchanged when
  ///   - any component of acc_b / gyro_b is non-finite ("non-finite imu sample"), or
  ///   - `s.t <= t` of the previous accepted sample ("imu timestamp not increasing").
  ///   No upper bound on dt is enforced; gap detection is the pipeline's job.
  /// Hot path: yes, per IMU sample. No logging or exceptions; no allocation on the success
  /// path and on error paths that use string literals.
  /// Tests: I-01 (static), I-02 (constant rotation),
  ///   I-03 (vs GTSAM ManifoldPreintegration < 1e-9),
  ///   I-04 (bias Jacobians vs numerical, < 1e-6), I-05 (covariance vs Monte Carlo).
  common::Status integrate(const common::ImuSample& s);

  /// Current accumulated delta, linearised at `bias_lin`. Reference valid until the next
  /// non-const call. Hot path safe. Tests: I-01..I-05.
  const common::PreintegratedDelta& delta() const;

  /// Predicts the state at the last integrated sample from `x0` (state at the first one):
  ///   dR', dv', dp' = delta corrected to first order for (x0.bias - bias_lin) with the
  ///                   bias Jacobians;
  ///   R1 = R0 * dR'
  ///   v1 = v0 + g_w * dt + R0 * dv'
  ///   p1 = p0 + v0 * dt + 0.5 * g_w * dt^2 + R0 * dp'
  ///   bias1 = x0.bias;  t1 = x0.t + (t_last - t_first) in integer ns.
  /// With no sample integrated yet (dt_s == 0) returns `x0` unchanged.
  /// Frames: x0, result: T_w_b and v_w_b in w; `g_w` gravity vector in w, m/s^2
  ///   (common::kGravityW or imu::InitState::g_w).
  /// Preconditions: x0.t equals the time of the first sample after reset().
  /// Errors: none. Hot path: yes (per scan); no allocation, logging or exceptions.
  /// Tests: I-01 (static: predicted state equals x0), I-02.
  common::NavState predict(const common::NavState& x0, const common::Vec3& g_w) const;

 private:
  common::PreintegratedDelta delta_;
  double acc_var_ = 0.0;         // (m/s^2)^2/Hz, acc_noise_density^2
  double gyro_var_ = 0.0;        // (rad/s)^2/Hz, gyro_noise_density^2
  common::Timestamp t_first_{};  // first accepted sample after reset()
  common::Timestamp t_prev_{};   // last accepted sample
  bool has_first_ = false;       // a sample has been accepted since reset()
};

}  // namespace mss::imu
