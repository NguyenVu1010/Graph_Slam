#pragma once

#include "mss/common/result.hpp"
#include "mss/common/types.hpp"
#include "mss/odom/config.hpp"

namespace mss::odom {

/// Integrates wheel odometry (forward speed + yaw rate) into relative body motion.
///
/// Model: planar unicycle in the wheel-odometry frame o (x forward, z up): `v_mps` along
/// x_o, `omega_radps` about z_o. Each sample k after the first uses ITS OWN reading held
/// over `dt = seconds_between(t_{k-1}, t_k)`, integrated exactly on SE(2) (constant
/// v, omega over dt) and embedded in SE(3) with z = roll = pitch = 0 in o.
/// The returned delta is expressed in the body frame b (decision 8):
/// `T_b0_b1 = T_b_o * T_o0_o1 * T_b_o^-1`.
/// Covariance: propagated to first order from per-sample noise. `speed_sigma_mps`
/// (sigma_v) and `yaw_rate_sigma_radps` (sigma_omega) are per-sample std devs; the
/// increment covariance per sample is `diag(sigma_v^2, sigma_omega^2) * dt^2` on the
/// (forward distance, yaw) increment, propagated through the SE(2) composition. The three
/// out-of-plane components (z, roll, pitch of o) get ZERO variance: cov is singular by
/// design; the backend odometry factor (O-01, O-02) must not invert it directly.
/// cov is expressed in the [rho, phi] right-perturbation tangent of T_b0_b1 (frame b1),
/// i.e. transformed from o with transform_covariance(cov_o, T_b_o).
/// `wheel_radius_m` and `track_m` are not used: WheelSample already carries v and omega.
/// Keyframe boundaries: keyframe time = scan `t_start`; deltas span the nearest wheel
/// sample times (take_delta chains at the last sample). The accepted mismatch is
/// `|t_delta - t_keyframe| <= 1.5 x` the observed wheel sample period; see
/// backend::Graph::add_odom_factor.
///
/// Ownership/threading: single-threaded; owned by pipeline::Estimator (front-end/backend
/// as ADR-005 assigns). No heap memory.
class WheelIntegrator {
 public:
  /// Errors: throws std::invalid_argument when `speed_sigma_mps < 0` or
  ///   `yaw_rate_sigma_radps < 0`.
  /// Frames: `T_b_o` = pose of the odometry frame o in b (frames.T_b_o, default identity).
  /// Hot path: no. Tests: O-01, O-02 construct it.
  WheelIntegrator(const WheelOdomConfig& cfg, const common::Pose& T_b_o);

  /// Integrates one sample (see class comment). The first sample ever only sets the
  /// start time.
  /// Units: m/s, rad/s, ns.
  /// Errors: returns Status::error and changes nothing when
  ///   - v_mps or omega_radps is non-finite ("non-finite wheel sample");
  ///   - `s.t <=` previous accepted sample time ("wheel timestamp not increasing").
  /// Hot path: yes, per wheel sample; no allocation, logging or exceptions.
  /// Tests: O-01 (motion consistent with odometry gives zero residual downstream).
  common::Status integrate(const common::WheelSample& s);

  /// Returns the motion accumulated since the previous take_delta() (or since the first
  /// sample) and starts a new accumulation at the last sample time, so consecutive
  /// deltas chain without gaps: `next.t0 == this.t1`.
  /// `t0` = start time, `t1` = time of the last integrated sample. With nothing
  /// integrated since the last call: identity, zero cov, `t0 == t1` (== last sample time,
  /// or Timestamp{} before any sample).
  /// Frames: T_b0_b1 = body at t1 expressed in body at t0 (b); cov as in class comment.
  /// Errors: none. Hot path: yes (per keyframe); no allocation.
  /// Tests: O-01, O-02.
  common::OdomDelta take_delta();
};

}  // namespace mss::odom
