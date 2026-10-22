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
  /// except along gravity; left to the backend); R_w_b = the rotation with yaw = 0 that
  /// maps mean(acc_b) onto +z of w (specific force at rest points up);
  /// g_w = [0, 0, -cfg.gravity_mps2].
  /// Frames: samples in b; result as documented in InitState (w gravity-aligned, z up).
  /// Units: s, m/s^2, rad/s.
  /// Preconditions: none checked beyond the errors below.
  /// Errors (Result::error):
  ///   - "empty window" when samples.size() < 2;
  ///   - "imu timestamp not increasing" when t is not strictly increasing;
  ///   - "window too short" when seconds_between(first.t, last.t) < init_window_s;
  ///   - "not static" when a std threshold is exceeded;
  ///   - "non-finite imu sample".
  /// Hot path: no (runs until it first succeeds, a few times per second); no allocation.
  /// Tests: I-01 (static gravity-only input yields R_w_b * mean(acc_b) along +z), imu
  ///   module unit tests.
  common::Result<InitState> try_init(common::Span<const common::ImuSample> samples);
};

}  // namespace mss::imu
