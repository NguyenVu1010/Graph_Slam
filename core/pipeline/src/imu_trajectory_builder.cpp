#include "imu_trajectory_builder.hpp"

#include <cstddef>
#include <optional>

namespace mss::pipeline::detail {

namespace {

// Entry index kept at output slot j when n entries are reduced to m (m < n), both ends
// kept: round(j * (n - 1) / (m - 1)) in integer arithmetic (pipeline.md section 2).
std::size_t kept_index(std::size_t j, std::size_t n, std::size_t m) {
  return (2 * j * (n - 1) + (m - 1)) / (2 * (m - 1));
}

}  // namespace

common::Status build_imu_trajectory(const ImuBuffer& imu,
                                    const common::NavState& x0,
                                    common::Timestamp t_end,
                                    const common::Vec3& g_w,
                                    imu::Preintegrator& scratch,
                                    imu::ImuTrajectory& traj) {
  traj.size = 0;
  traj.t_start = x0.t;
  const std::optional<common::ImuSample> s0 = imu.interpolate(x0.t);
  if (!s0) {
    return common::Status::error("imu does not cover scan start");
  }
  if (imu.back().t < t_end) {
    return common::Status::error("imu does not cover scan end");
  }
  // Raw samples (t_start, t_last], t_last = first sample with t >= t_end.
  const std::size_t i_first = imu.first_after(x0.t);
  std::size_t i_last = i_first;
  while (i_last < imu.size() && imu.at(i_last).t < t_end) {
    ++i_last;
  }
  // i_last < size because back().t >= t_end; when t_end <= t_start no raw sample is
  // needed beyond s0 (i_last stays at i_first only if that sample has t >= t_end).
  const std::size_t n_raw = i_last < imu.size() ? i_last - i_first + 1 : 0;
  const std::size_t n_entries = 1 + n_raw;
  constexpr std::size_t kMax = imu::ImuTrajectory::kMaxImuPerScan;

  scratch.reset(x0.bias);
  if (!scratch.integrate(*s0).is_ok()) {
    return common::Status::error("imu sample rejected");
  }
  traj.samples[0].t = x0.t;
  traj.samples[0].T_b0_b = common::Pose{};  // exactly the identity
  std::size_t out = 1;
  const common::Pose T_b0_w = x0.T_w_b.inverse();
  for (std::size_t e = 1; e < n_entries; ++e) {
    const common::ImuSample& s = imu.at(i_first + e - 1);
    if (!scratch.integrate(s).is_ok()) {
      traj.size = 0;
      return common::Status::error("imu sample rejected");
    }
    const bool keep = n_entries <= kMax || kept_index(out, n_entries, kMax) == e;
    if (keep) {
      const common::NavState x = scratch.predict(x0, g_w);
      traj.samples[out].t = s.t;
      traj.samples[out].T_b0_b = T_b0_w * x.T_w_b;
      ++out;
    }
  }
  traj.size = out;
  return common::Status::ok();
}

common::NavState predict_state_at(const ImuBuffer& imu,
                                  const imu::Preintegrator& pred,
                                  common::Timestamp pred_last_t,
                                  const common::NavState& x0,
                                  common::Timestamp t,
                                  const common::Vec3& g_w,
                                  imu::Preintegrator& scratch) {
  scratch = pred;
  for (std::size_t i = imu.first_after(pred_last_t); i < imu.size() && imu.at(i).t < t; ++i) {
    (void)scratch.integrate(imu.at(i));
  }
  if (t > pred_last_t) {
    const std::optional<common::ImuSample> s = imu.interpolate(t);
    if (s) {
      (void)scratch.integrate(*s);
    }
  }
  return scratch.predict(x0, g_w);
}

}  // namespace mss::pipeline::detail
