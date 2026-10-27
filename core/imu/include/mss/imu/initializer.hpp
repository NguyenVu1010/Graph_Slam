#pragma once

#include "mss/common/result.hpp"
#include "mss/common/span.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"

namespace mss::imu {

/// Result of static IMU initialisation.
struct InitState {
  common::Vec3 g_w = common::Vec3::Zero();        ///< m/s^2, gravity in w: [0, 0, -gravity_mps2]
  common::ImuBias bias;                           ///< bg = mean gyro (rad/s); ba = 0 (m/s^2)
  common::Mat3 R_w_b = common::Mat3::Identity();  ///< initial body attitude, yaw = 0
  /// Mean accelerometer reading of the window (m/s^2, in b): the specific force at rest,
  /// `ba - R_w_b^T g_w` (the static-gravity factor of backend_factors.md section 6c).
  common::Vec3 acc_mean_b = common::Vec3::Zero();
  /// Covariance of acc_mean_b, (m/s^2)^2: diag(max(s_k^2, acc_noise_density^2 rate_hz)) / n,
  /// s_k^2 the sample variance of axis k over the n samples (the believed white noise as a
  /// floor).
  common::Mat3 acc_mean_cov = common::Mat3::Zero();
};

/// Static-window initialisation of gravity direction, gyro bias and attitude.
///
/// Ownership/threading: single-threaded; owned by pipeline::Estimator, used on the
/// front-end thread before the first keyframe. Stateless between calls.
class ImuInitializer {
 public:
  /// Validates `cfg`.
  /// Errors: throws std::invalid_argument when `init_window_s <= 0`,
  ///   `init_max_acc_std_mps2 <= 0`, `init_max_gyro_std_radps <= 0` or
  ///   `gravity_mps2 <= 0`.
  /// Hot path: no. Tests: initializer unit tests (imu module).
  explicit ImuInitializer(const ImuConfig& cfg);

  /// Tries to initialise from `samples` (ALL of them are used; the caller passes the
  /// window, e.g. the last init_window_s of buffered IMU).
  /// Accepts the window as static when, for every axis, the sample standard deviation
  /// of acc_b is <= init_max_acc_std_mps2 and of gyro_b is <= init_max_gyro_std_radps.
  /// Then: bias.bg = mean(gyro_b); bias.ba = 0 (not observable from a static window
  /// except along gravity; left to the backend); with `a = mean(acc_b)`:
  /// `R_w_b = Ry(pitch) * Rx(roll)` (ZYX with yaw = 0), `roll = atan2(a_y, a_z)`,
  /// `pitch = atan2(-a_x, hypot(a_y, a_z))`, so R_w_b * a points along +z of w
  /// (specific force at rest points up);
  /// g_w = [0, 0, -cfg.gravity_mps2]; acc_mean_b = a and acc_mean_cov as documented in
  /// InitState (the window statistics for backend::Graph::add_static_gravity_factor).
  /// Frames: samples in b; result as documented in InitState (w gravity-aligned, z up).
  /// Units: s, m/s^2, rad/s.
  /// Preconditions: none checked beyond the errors below.
  /// Errors (Result::error). Checks run in the listed order; the first failure is
  ///   returned:
  ///   1. "empty window" when samples.size() < 2;
  ///   2. "non-finite imu sample";
  ///   3. "imu timestamp not increasing" when t is not strictly increasing;
  ///   4. "window too short" when seconds_between(first.t, last.t) < init_window_s;
  ///   5. "not static" when a std threshold is exceeded.
  /// Hot path: no (runs until it first succeeds, a few times per second); no allocation
  /// on the success path and on error paths that use string literals.
  /// Tests: I-01 (static gravity-only input yields R_w_b * mean(acc_b) along +z), imu
  ///   module unit tests.
  common::Result<InitState> try_init(common::Span<const common::ImuSample> samples);

 private:
  ImuConfig cfg_;
};

}  // namespace mss::imu
