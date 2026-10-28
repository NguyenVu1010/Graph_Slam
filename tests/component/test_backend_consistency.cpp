// Backend consistency with an unobserved axis (PR 4e; backend_factors.md section 5,
// imu_only_observability.md "Exploratory controls"). A planar circle, a noisy IMU chain and
// LiDAR relative-pose factors that carry no information along world x (covariance
// cov + (1 km)^2 d d^T, d = world x in b_j, the study's "x removed" hook), with the
// measurement drawn from that covariance: the LiDAR residual along d reaches hundreds of
// metres. The keyframe guesses come from the IMU prediction off the previous estimate (the
// pipeline's rule) and optimize() runs once per keyframe. Checked: world-x NEES of each new
// keyframe against the marginal the backend reports, averaged over the run and 50 seeds.
//
// Each parametrisation draws the measurement noise in the chart of the factor form under
// test, so the test checks the factor and the optimiser (the estimate is consistent with the
// marginal), not one chart against the other. All forms read the same covariance to first
// order; they extend it differently at large residuals (backend_factors.md section 5):
//   - se3_exact: Z = Z_true Exp(n) (the SE(3) residual is -n at the truth). Proves that with
//     exact Jacobians one Gauss-Newton step per keyframe stays consistent (NEES in [0.5, 2]).
//   - decoupled: R_Z = R_true Exp(-n_phi), p_Z = t_true - R_Z n_rho (the decoupled residual is
//     n at the truth). Proves the same for the decoupled form.
//   - between (control, the default before ADR-021): the SE(3) noise as for se3_exact; its Jacobian
//     omits Jr^-1, and the run fails or leaves [0.5, 2]. Documents the Jacobian defect. Which
//     chart keeps a free translation axis free (decoupled) is shown analytically in section 5
//     and by the study's acceptance control, not by this test.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include "mss/backend/graph.hpp"
#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/preintegrator.hpp"

#include "backend_test_config.hpp"

namespace mss::backend {
namespace {

using common::Mat3;
using common::Mat6;
using common::NavState;
using common::Vec3;
using common::Vec6;

constexpr int kSeeds = 50;
constexpr int kKeyframes = 25;
constexpr int kSamplesPerKf = 200;  // 1 s at 200 Hz
constexpr double kSpeedMps = 2.0;   // circle speed
constexpr double kYawRateRadps = 0.3;
constexpr double kLidarPosSigmaM = 0.02;
constexpr double kLidarRotSigmaRad = 0.002;
constexpr double kFreeSigmaM = 1000.0;  // along world x: no information (study hook)

Vec3 normal3(std::mt19937& rng, double sigma) {
  std::normal_distribution<double> n(0.0, sigma);
  const double x = n(rng);
  const double y = n(rng);
  const double z = n(rng);
  return Vec3(x, y, z);
}

// Mean NEES of the world-x position of each new keyframe (k >= 1) over one run.
struct RunNees {
  double sum = 0.0;
  int n = 0;
};

RunNees run_circle(std::uint32_t seed, RelativeFactor form) {
  const imu::ImuConfig imu_cfg = test::default_imu_config();
  BackendConfig cfg = test::default_backend_config();
  cfg.relative_factor = form;
  const double dt_s = 1.0 / imu_cfg.rate_hz;
  const auto dt_ns = static_cast<std::int64_t>(std::llround(dt_s * 1e9));
  const double sa = imu_cfg.acc_noise_density / std::sqrt(dt_s);   // m/s^2 per sample
  const double sg = imu_cfg.gyro_noise_density / std::sqrt(dt_s);  // rad/s per sample
  const Vec3 g_w(0.0, 0.0, -imu_cfg.gravity_mps2);
  std::mt19937 rng(seed);

  // Truth: level circle, constant body rates; integrated with the same Preintegrator
  // (noise-free samples), so the truth is exact for the IMU model.
  common::ImuSample clean;
  clean.gyro_b = Vec3(0.0, 0.0, kYawRateRadps);
  clean.acc_b = Vec3(0.0, kYawRateRadps * kSpeedMps, imu_cfg.gravity_mps2);
  NavState truth0;
  truth0.t = common::Timestamp{1'000'000'000};
  truth0.v_w_b = Vec3(kSpeedMps, 0.0, 0.0);

  // Keyframe-0 guess drawn from the keyframe-0 priors (they are centred on it).
  NavState guess0 = truth0;
  Vec6 d0;
  d0 << normal3(rng, cfg.prior_pos_sigma_m), normal3(rng, cfg.prior_rot_sigma_rad);
  guess0.T_w_b = common::boxplus(truth0.T_w_b, d0);
  guess0.v_w_b += normal3(rng, cfg.prior_vel_sigma_mps);
  guess0.bias.ba = normal3(rng, cfg.prior_bias_acc_sigma_mps2);
  guess0.bias.bg = normal3(rng, cfg.prior_bias_gyro_sigma_radps);

  Graph g(cfg, imu_cfg, Vec3::Zero());
  g.add_keyframe(truth0.t, guess0);
  EXPECT_TRUE(g.optimize().is_ok());
  NavState est = guess0;
  NavState truth = truth0;
  Mat6 lidar_cov = Mat6::Zero();
  lidar_cov.diagonal() << Vec3::Constant(kLidarPosSigmaM * kLidarPosSigmaM),
      Vec3::Constant(kLidarRotSigmaRad * kLidarRotSigmaRad);
  RunNees out;
  for (int k = 1; k < kKeyframes; ++k) {
    imu::Preintegrator clean_pre(imu_cfg);
    imu::Preintegrator noisy_pre(imu_cfg);
    clean_pre.reset(common::ImuBias{});
    noisy_pre.reset(est.bias);
    for (int s = 0; s <= kSamplesPerKf; ++s) {
      clean.t = common::Timestamp{truth.t.ns + s * dt_ns};
      common::ImuSample noisy = clean;
      if (s > 0) {  // the first sample only sets the start time
        noisy.acc_b += normal3(rng, sa);
        noisy.gyro_b += normal3(rng, sg);
      }
      EXPECT_TRUE(clean_pre.integrate(clean).is_ok());
      EXPECT_TRUE(noisy_pre.integrate(noisy).is_ok());
    }
    const NavState truth_j = clean_pre.predict(truth, g_w);
    const NavState guess_j = noisy_pre.predict(est, g_w);  // the pipeline's guess

    // LiDAR noise n ~ N(0, cov) ([rho, phi]), cov in b_j with (1 km)^2 along d, applied in
    // the chart of the form (file comment).
    const Vec3 d_b = truth_j.T_w_b.R.transpose() * Vec3::UnitX();
    Mat6 cov = lidar_cov;
    cov.topLeftCorner<3, 3>() += kFreeSigmaM * kFreeSigmaM * d_b * d_b.transpose();
    const Mat6 L = cov.llt().matrixL();
    Vec6 u;
    u << normal3(rng, 1.0), normal3(rng, 1.0);
    const Vec6 n = L * u;
    const common::Pose Z_true = truth.T_w_b.inverse() * truth_j.T_w_b;
    common::Pose Z = Z_true * common::se3_exp(n);
    if (form == RelativeFactor::kDecoupled) {
      Z.R = Z_true.R * common::so3_exp(-n.tail<3>());
      Z.p = Z_true.p - Z.R * n.head<3>();
    }

    const common::KeyId j = g.add_keyframe(truth_j.t, guess_j);
    g.add_imu_factor(j - 1, j, noisy_pre.delta());
    g.add_lidar_factor(j - 1, j, Z, cov, nullptr);
    const common::Result<common::Estimate> r = g.optimize();
    if (!r.is_ok()) {
      return out;
    }
    est = r.value().state;
    const Mat3 cov_p_w =
        est.T_w_b.R * r.value().cov.topLeftCorner<3, 3>() * est.T_w_b.R.transpose();
    const double ex = est.T_w_b.p.x() - truth_j.T_w_b.p.x();
    out.sum += ex * ex / cov_p_w(0, 0);
    ++out.n;
    truth = truth_j;
  }
  return out;
}

// Mean world-x NEES over the seeds; `ok` false when an optimize() failed.
std::tuple<double, double, bool> nees_over_seeds(RelativeFactor form) {
  double sum = 0.0;
  int n = 0;
  double worst_run = 0.0;
  bool ok = true;
  for (int s = 0; s < kSeeds; ++s) {
    const RunNees r = run_circle(static_cast<std::uint32_t>(1000 + s), form);
    ok = ok && r.n == kKeyframes - 1;
    sum += r.sum;
    n += r.n;
    worst_run = std::max(worst_run, r.sum / std::max(1, r.n));
    if (!ok) {
      break;
    }
  }
  return {sum / std::max(1, n), worst_run, ok};
}

class UnobservedAxis : public ::testing::TestWithParam<RelativeFactor> {};

TEST_P(UnobservedAxis, EstimateConsistentWithMarginal) {
  const auto [nees, worst_run, ok] = nees_over_seeds(GetParam());  // expectation 1 (1-D)
  RecordProperty("nees_x_mean", std::to_string(nees));
  RecordProperty("nees_x_worst_run_mean", std::to_string(worst_run));
  EXPECT_TRUE(ok);
  EXPECT_GE(nees, 0.5);
  EXPECT_LE(nees, 2.0);
}

INSTANTIATE_TEST_SUITE_P(BackendComponent,
                         UnobservedAxis,
                         ::testing::Values(RelativeFactor::kSe3Exact, RelativeFactor::kDecoupled),
                         [](const ::testing::TestParamInfo<RelativeFactor>& info) {
                           return info.param == RelativeFactor::kSe3Exact
                                      ? std::string("se3_exact")
                                      : std::string("decoupled");
                         });

// Control: the between form under the SE(3) noise is not consistent (fails or leaves [0.5, 2]).
TEST(BackendComponent, UnobservedAxisBetweenFactorControlIsInconsistent) {
  const auto [nees, worst_run, ok] = nees_over_seeds(RelativeFactor::kBetween);
  RecordProperty("nees_x_mean", std::to_string(nees));
  RecordProperty("completed", ok ? "true" : "false");
  EXPECT_TRUE(!ok || nees < 0.5 || nees > 2.0);
}

}  // namespace
}  // namespace mss::backend
