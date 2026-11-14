#include "mss2d/motion.hpp"

#include <cmath>

namespace mss2d {

namespace {

/// Bounds the work of one integrate() call; beyond it the sub-step grows. Reached only
/// after a data gap of more than kMaxIntegrationSteps * integration_step_s (5 s by default).
constexpr std::int64_t kMaxIntegrationSteps = 1000;

constexpr double kSecondsPerNs = 1e-9;

float seconds(std::int64_t ns) {
  return static_cast<float>(static_cast<double>(ns) * kSecondsPerNs);
}

float lerp(float a, float b, float f) {
  return a + f * (b - a);
}

/// Index of the first of n time-sorted entries with time >= t (n when none). `time_of(i)`
/// returns the i-th oldest entry's time.
template <typename TimeOf>
int lower_bound(int n, Timestamp t, TimeOf time_of) {
  int lo = 0;
  int hi = n;
  while (lo < hi) {
    const int mid = (lo + hi) / 2;
    if (time_of(mid) < t) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return lo;
}

}  // namespace

MotionModel::MotionModel(const MotionConfig& cfg)
    : cfg_(cfg),
      bias_(cfg.gyro_bias_rad_s),
      wheel_timeout_ns_(static_cast<std::int64_t>(static_cast<double>(cfg.wheel_timeout_s) *
                                                  static_cast<double>(kNanosecondsPerSecond))),
      step_ns_(static_cast<std::int64_t>(static_cast<double>(cfg.integration_step_s) *
                                         static_cast<double>(kNanosecondsPerSecond))) {
  if (step_ns_ < 1) {
    step_ns_ = 1;
  }
}

const ImuSample2d& MotionModel::imu_at(int i) const {
  return imu_[static_cast<std::size_t>((imu_head_ - n_imu_ + i + kMotionImuHistory) %
                                       kMotionImuHistory)];
}

const WheelSample& MotionModel::wheel_at(int i) const {
  return wheel_[static_cast<std::size_t>((wheel_head_ - n_wheel_ + i + kMotionWheelHistory) %
                                         kMotionWheelHistory)];
}

void MotionModel::add_imu(const ImuSample2d& s) {
  if (n_imu_ > 0 && s.t <= imu_at(n_imu_ - 1).t) {
    return;
  }
  imu_[static_cast<std::size_t>(imu_head_)] = s;
  imu_head_ = (imu_head_ + 1) % kMotionImuHistory;
  if (n_imu_ < kMotionImuHistory) {
    ++n_imu_;
  }
}

void MotionModel::add_wheel(const WheelSample& s) {
  if (n_wheel_ > 0 && s.t <= wheel_at(n_wheel_ - 1).t) {
    return;
  }
  wheel_[static_cast<std::size_t>(wheel_head_)] = s;
  wheel_head_ = (wheel_head_ + 1) % kMotionWheelHistory;
  if (n_wheel_ < kMotionWheelHistory) {
    ++n_wheel_;
  }
}

void MotionModel::set_constant_velocity(float vx_m_s, float vy_m_s, float w_rad_s) {
  cv_vx_ = vx_m_s;
  cv_vy_ = vy_m_s;
  cv_w_ = w_rad_s;
}

void MotionModel::set_gyro_bias(float b_rad_s) {
  const float m = cfg_.max_gyro_bias_rad_s;
  bias_ = b_rad_s > m ? m : (b_rad_s < -m ? -m : b_rad_s);
}

float MotionModel::yaw_rate(Timestamp t) const {
  if (n_imu_ == 0) {
    return cv_w_;
  }
  const int i = lower_bound(n_imu_, t, [this](int k) { return imu_at(k).t; });
  float g = 0.0F;
  if (i == 0) {
    g = imu_at(0).gyro_z_rad_s;
  } else if (i == n_imu_) {
    g = imu_at(n_imu_ - 1).gyro_z_rad_s;
  } else {
    const ImuSample2d& a = imu_at(i - 1);
    const ImuSample2d& b = imu_at(i);
    const float f = static_cast<float>(t.ns - a.t.ns) / static_cast<float>(b.t.ns - a.t.ns);
    g = lerp(a.gyro_z_rad_s, b.gyro_z_rad_s, f);
  }
  return g - bias_;
}

bool MotionModel::velocity(Timestamp t, float& vx_m_s, float& vy_m_s) const {
  vx_m_s = cv_vx_;
  vy_m_s = cv_vy_;
  if (!cfg_.use_wheel || n_wheel_ == 0) {
    return false;
  }
  const int i = lower_bound(n_wheel_, t, [this](int k) { return wheel_at(k).t; });
  float v = 0.0F;
  std::int64_t gap_ns = 0;
  if (i == 0) {
    v = wheel_at(0).v_m_s;
    gap_ns = wheel_at(0).t.ns - t.ns;
  } else if (i == n_wheel_) {
    v = wheel_at(n_wheel_ - 1).v_m_s;
    gap_ns = t.ns - wheel_at(n_wheel_ - 1).t.ns;
  } else {
    const WheelSample& a = wheel_at(i - 1);
    const WheelSample& b = wheel_at(i);
    const std::int64_t da = t.ns - a.t.ns;
    const std::int64_t db = b.t.ns - t.ns;
    gap_ns = da < db ? da : db;
    v = lerp(a.v_m_s, b.v_m_s, static_cast<float>(da) / static_cast<float>(b.t.ns - a.t.ns));
  }
  if (gap_ns > wheel_timeout_ns_) {
    return false;
  }
  vx_m_s = v;
  vy_m_s = 0.0F;
  return true;
}

Pose2 MotionModel::integrate(Timestamp t0, Timestamp t1) const {
  const std::int64_t dt_ns = t1.ns - t0.ns;
  if (dt_ns <= 0) {
    return Pose2();
  }
  std::int64_t n = (dt_ns + step_ns_ - 1) / step_ns_;
  n = n > kMaxIntegrationSteps ? kMaxIntegrationSteps : n;
  const float h = seconds(dt_ns) / static_cast<float>(n);
  Pose2 T;
  for (std::int64_t k = 0; k < n; ++k) {
    const Timestamp tm{t0.ns + (2 * k + 1) * dt_ns / (2 * n)};  // sub-step midpoint
    float vx = 0.0F;
    float vy = 0.0F;
    velocity(tm, vx, vy);
    Vec3f d;
    d[0] = vx * h;
    d[1] = vy * h;
    d[2] = yaw_rate(tm) * h;
    T = T * Pose2::exp(d);
  }
  return T;
}

void DeskewTable::build(
    const MotionModel& motion, Timestamp t_start, Timestamp t_end, int knots, const Pose2& T_b_l) {
  knots_ = knots < 1 ? 1 : (knots > kMaxDeskewKnots ? kMaxDeskewKnots : knots);
  span_ns_ = t_end.ns - t_start.ns;
  if (span_ns_ <= 0) {
    span_ns_ = 0;
    for (int k = 0; k <= knots_; ++k) {
      T_le_l_[static_cast<std::size_t>(k)] = Pose2();
    }
    return;
  }
  // T_b0_bk, cumulative from the scan start; stored in place, then re-expressed at the end.
  T_le_l_[0] = Pose2();
  Timestamp prev = t_start;
  for (int k = 1; k <= knots_; ++k) {
    const Timestamp tk{t_start.ns + span_ns_ * k / knots_};
    T_le_l_[static_cast<std::size_t>(k)] =
        T_le_l_[static_cast<std::size_t>(k - 1)] * motion.integrate(prev, tk);
    prev = tk;
  }
  const Pose2 T_be_b0 = T_le_l_[static_cast<std::size_t>(knots_)].inverse();
  const Pose2 T_l_b = T_b_l.inverse();
  for (int k = 0; k <= knots_; ++k) {
    Pose2& T = T_le_l_[static_cast<std::size_t>(k)];
    T = T_l_b * (T_be_b0 * T) * T_b_l;
  }
}

Pose2 DeskewTable::at(std::int32_t t_offset_ns) const {
  if (span_ns_ <= 0) {
    return T_le_l_[0];
  }
  float u =
      static_cast<float>(t_offset_ns) / static_cast<float>(span_ns_) * static_cast<float>(knots_);
  const float hi = static_cast<float>(knots_);
  u = u < 0.0F ? 0.0F : (u > hi ? hi : u);
  int i = static_cast<int>(u);
  i = i >= knots_ ? knots_ - 1 : i;
  const float f = u - static_cast<float>(i);
  const Pose2& a = T_le_l_[static_cast<std::size_t>(i)];
  const Pose2& b = T_le_l_[static_cast<std::size_t>(i + 1)];
  return Pose2(lerp(a.x, b.x, f), lerp(a.y, b.y, f), a.yaw + f * normalize_angle(b.yaw - a.yaw));
}

void deskew_scan(const Scan2d& scan,
                 const DeskewTable& table,
                 float min_range_m,
                 float max_range_m,
                 PointCloud2d& out) {
  out.count = 0;
  for (std::uint32_t i = 0; i < scan.count && i < kMaxScanPoints; ++i) {
    const ScanPoint2d& sp = scan.points[i];
    if (!(sp.range_m >= min_range_m && sp.range_m <= max_range_m && sp.range_m > 0.0F)) {
      continue;
    }
    const Vec2f p_l{sp.range_m * std::cos(sp.angle_rad), sp.range_m * std::sin(sp.angle_rad)};
    out.pts[out.count++] = table.at(sp.t_offset_ns) * p_l;
  }
}

}  // namespace mss2d
