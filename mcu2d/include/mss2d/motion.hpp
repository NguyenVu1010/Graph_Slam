#pragma once

#include <array>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/messages.hpp"
#include "mss2d/points2d.hpp"
#include "mss2d/pose2.hpp"
#include "mss2d/timestamp.hpp"

namespace mss2d {

/// Body motion between two times from the recent gyro and wheel samples
/// (docs/frontend.md section 2). Frame b = body = IMU; the wheel speed is the forward speed
/// of the body origin along b.x (no side slip).
///
///   yaw rate  w(t) = gyro_z(t) - bias                         (linear between samples)
///   velocity  v(t) = (wheel(t), 0) if a wheel sample is within wheel_timeout_s of t,
///                    else the constant body velocity set by set_constant_velocity()
///   T_b0_b1 = prod_k Exp([v_x h, v_y h, w h])                 (midpoint of each sub-step h)
///
/// Samples older than the history are forgotten; queries outside the stored span use the
/// nearest sample. Fixed storage, no allocation, single-threaded (main loop).
class MotionModel {
 public:
  /// `cfg` must have passed MotionConfig::validate().
  explicit MotionModel(const MotionConfig& cfg);

  /// Samples must arrive in increasing time (the merger guarantees it); others are ignored.
  void add_imu(const ImuSample2d& s);
  void add_wheel(const WheelSample& s);

  /// Body twist used when no wheel speed is available: (vx, vy) m/s in b. The yaw rate
  /// comes from the gyro when there is one, else from w_rad_s.
  void set_constant_velocity(float vx_m_s, float vy_m_s, float w_rad_s);

  float gyro_bias() const { return bias_; }
  /// Clamped to +-max_gyro_bias_rad_s.
  void set_gyro_bias(float b_rad_s);

  bool has_imu() const { return n_imu_ > 0; }

  /// Bias-corrected yaw rate at t, rad/s.
  float yaw_rate(Timestamp t) const;
  /// Body velocity (vx, vy) at t, m/s, and whether it came from the wheel.
  bool velocity(Timestamp t, float& vx_m_s, float& vy_m_s) const;

  /// T_b0_b1: body pose at t1 in the body frame at t0 (identity when t1 <= t0).
  Pose2 integrate(Timestamp t0, Timestamp t1) const;

 private:
  const ImuSample2d& imu_at(int i) const;  ///< i-th oldest stored sample
  const WheelSample& wheel_at(int i) const;

  MotionConfig cfg_;
  float bias_;
  float cv_vx_ = 0.0F;
  float cv_vy_ = 0.0F;
  float cv_w_ = 0.0F;
  std::int64_t wheel_timeout_ns_;
  std::int64_t step_ns_;
  std::array<ImuSample2d, kMotionImuHistory> imu_{};
  int imu_head_ = 0;  ///< next write index
  int n_imu_ = 0;
  std::array<WheelSample, kMotionWheelHistory> wheel_{};
  int wheel_head_ = 0;
  int n_wheel_ = 0;
};

/// Body poses over one LiDAR turn, tabulated at deskew_knots + 1 equally spaced times from
/// t_start to t_end and expressed in the LiDAR frame at t_end:
///   T_le_l(t) = T_l_b * T_be_b(t) * T_b_l,
/// linear in (x, y, yaw) between knots. Fixed storage.
class DeskewTable {
 public:
  /// Fills the table for [t_start, t_end] from `motion`. With t_end <= t_start every knot is
  /// the identity.
  void build(
      const MotionModel& motion, Timestamp t_start, Timestamp t_end, int knots, const Pose2& T_b_l);

  /// T_le_l at t_offset_ns after t_start (clamped to the turn).
  Pose2 at(std::int32_t t_offset_ns) const;

 private:
  std::array<Pose2, kMaxDeskewKnots + 1> T_le_l_{};
  int knots_ = 0;
  std::int64_t span_ns_ = 0;
};

/// Polar -> Cartesian in the LiDAR frame at the scan end: each kept beam (same range filter
/// as scan_to_points) is moved by T_le_l(t) of its sample time.
void deskew_scan(const Scan2d& scan,
                 const DeskewTable& table,
                 float min_range_m,
                 float max_range_m,
                 PointCloud2d& out);

}  // namespace mss2d
