#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "mss/common/timestamp.hpp"

namespace mss::pipeline::detail {

/// ADR-023 / backlog G9 (gnss.md section 5.3): the signed wheel odometer the track-yaw aligner
/// reads, a trapezoid of the wheel speed between consecutive finite samples, and the bound of the
/// travel it interpolates across sample gaps.
///
/// The nominal sample period is the mean of the regular intervals so far (the first interval
/// starts it). An interval longer than `gap_periods` nominal periods is a gap (a driver silent
/// while the robot stops, a lost message): its trapezoid is an interpolation. With the speed
/// within [0, max(|v0|, |v1|)] and no reversal inside it, the travel over the excess of the gap
/// over one nominal period is off by at most `max(|v0|, |v1|) x (dt - nominal)`; that is summed
/// in gap_bound_m(). A regular interval (jitter included) adds nothing, at any sample rate.
///
/// Internal to the pipeline; single-threaded (EstimatorCore). No allocation.
class WheelOdometer {
 public:
  /// `gap_periods`: GnssConfig::align_yaw_wheel_gap_periods (> 1; +inf: never a gap).
  explicit WheelOdometer(double gap_periods = std::numeric_limits<double>::infinity())
      : gap_periods_(gap_periods) {}

  /// Adds a finite speed sample at `t` (later than the last one; the caller checks both).
  /// Returns true when the interval since the last sample was a gap.
  bool push(common::Timestamp t, double v_mps) {
    if (!have_) {
      have_ = true;
      last_t_ = t;
      last_v_mps_ = v_mps;
      return false;
    }
    const double dt_s = common::seconds_between(last_t_, t);
    reading_m_ += 0.5 * (last_v_mps_ + v_mps) * dt_s;
    const bool gap = n_regular_ > 0 && dt_s > gap_periods_ * nominal_s_;
    if (gap) {
      gap_bound_m_ += std::max(std::abs(last_v_mps_), std::abs(v_mps)) * (dt_s - nominal_s_);
      ++n_gaps_;
    } else {
      ++n_regular_;
      nominal_s_ += (dt_s - nominal_s_) / static_cast<double>(n_regular_);
    }
    last_t_ = t;
    last_v_mps_ = v_mps;
    return gap;
  }

  [[nodiscard]] bool has_reading() const { return have_; }
  [[nodiscard]] common::Timestamp last_t() const { return last_t_; }
  /// m, signed travel since the first sample.
  [[nodiscard]] double reading_m() const { return reading_m_; }
  /// m, sum of the interpolation bounds of the gaps so far.
  [[nodiscard]] double gap_bound_m() const { return gap_bound_m_; }
  /// s, mean of the regular intervals (0 before the second sample).
  [[nodiscard]] double nominal_period_s() const { return nominal_s_; }
  [[nodiscard]] std::uint64_t n_gaps() const { return n_gaps_; }

 private:
  double gap_periods_;
  bool have_ = false;
  common::Timestamp last_t_{};
  double last_v_mps_ = 0.0;
  double reading_m_ = 0.0;
  double gap_bound_m_ = 0.0;
  double nominal_s_ = 0.0;
  std::uint64_t n_regular_ = 0;
  std::uint64_t n_gaps_ = 0;
};

}  // namespace mss::pipeline::detail
