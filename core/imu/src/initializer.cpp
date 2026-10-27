#include "mss/imu/initializer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"

namespace mss::imu {

using common::ImuSample;
using common::Mat3;
using common::Result;
using common::Span;
using common::Vec3;

ImuInitializer::ImuInitializer(const ImuConfig& cfg) : cfg_(cfg) {
  // `!(x > 0)` also rejects NaN.
  if (!(cfg.init_window_s > 0.0)) {
    throw std::invalid_argument("imu.init_window_s must be > 0");
  }
  if (!(cfg.init_max_acc_std_mps2 > 0.0)) {
    throw std::invalid_argument("imu.init_max_acc_std_mps2 must be > 0");
  }
  if (!(cfg.init_max_gyro_std_radps > 0.0)) {
    throw std::invalid_argument("imu.init_max_gyro_std_radps must be > 0");
  }
  if (!(cfg.gravity_mps2 > 0.0)) {
    throw std::invalid_argument("imu.gravity_mps2 must be > 0");
  }
}

Result<InitState> ImuInitializer::try_init(Span<const ImuSample> samples) {
  // Checks in the header's documented order; string literals only (no allocation).
  if (samples.size() < 2) {
    return Result<InitState>::error("empty window");
  }
  for (const ImuSample& s : samples) {
    if (!s.acc_b.allFinite() || !s.gyro_b.allFinite()) {
      return Result<InitState>::error("non-finite imu sample");
    }
  }
  for (std::size_t k = 1; k < samples.size(); ++k) {
    if (samples[k].t <= samples[k - 1].t) {
      return Result<InitState>::error("imu timestamp not increasing");
    }
  }
  const double span_s = common::seconds_between(samples[0].t, samples[samples.size() - 1].t);
  if (span_s < cfg_.init_window_s) {
    return Result<InitState>::error("window too short");
  }

  // Two passes: mean, then sample (n - 1) variance per axis.
  const auto n = static_cast<double>(samples.size());
  Vec3 acc_mean = Vec3::Zero();   // m/s^2
  Vec3 gyro_mean = Vec3::Zero();  // rad/s
  for (const ImuSample& s : samples) {
    acc_mean += s.acc_b;
    gyro_mean += s.gyro_b;
  }
  acc_mean /= n;
  gyro_mean /= n;
  Vec3 acc_ss = Vec3::Zero();   // sum of squared deviations, (m/s^2)^2
  Vec3 gyro_ss = Vec3::Zero();  // (rad/s)^2
  for (const ImuSample& s : samples) {
    const Vec3 da = s.acc_b - acc_mean;
    const Vec3 dg = s.gyro_b - gyro_mean;
    acc_ss += da.cwiseProduct(da);
    gyro_ss += dg.cwiseProduct(dg);
  }
  const double max_acc_var = cfg_.init_max_acc_std_mps2 * cfg_.init_max_acc_std_mps2;
  const double max_gyro_var = cfg_.init_max_gyro_std_radps * cfg_.init_max_gyro_std_radps;
  if (acc_ss.maxCoeff() / (n - 1.0) > max_acc_var ||
      gyro_ss.maxCoeff() / (n - 1.0) > max_gyro_var) {
    return Result<InitState>::error("not static");
  }

  // R_w_b = Ry(pitch) * Rx(roll), yaw = 0, so R_w_b * acc_mean points along +z of w.
  const double roll = std::atan2(acc_mean.y(), acc_mean.z());                              // rad
  const double pitch = std::atan2(-acc_mean.x(), std::hypot(acc_mean.y(), acc_mean.z()));  // rad
  InitState out;
  out.R_w_b = common::so3_exp(Vec3(0.0, pitch, 0.0)) * common::so3_exp(Vec3(roll, 0.0, 0.0));
  out.bias.bg = gyro_mean;
  out.bias.ba = Vec3::Zero();
  out.g_w = Vec3(0.0, 0.0, -cfg_.gravity_mps2);
  out.acc_mean_b = acc_mean;
  // Variance of the mean per axis; the believed white noise per sample is the floor.
  const double noise_var = cfg_.acc_noise_density * cfg_.acc_noise_density * cfg_.rate_hz;
  for (int k = 0; k < 3; ++k) {
    out.acc_mean_cov(k, k) = std::max(acc_ss(k) / (n - 1.0), noise_var) / n;  // (m/s^2)^2
  }
  return Result<InitState>::ok(out);
}

}  // namespace mss::imu
