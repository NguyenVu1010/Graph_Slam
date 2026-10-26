// Cross-check of mss::imu::Preintegrator against GTSAM (I-03).
// Derivation: docs/derivations/imu_preintegration.md (§6 explains the
// GTSAM_TANGENT_PREINTEGRATION note). GTSAM is used only in this test binary.
#include <cstddef>
#include <random>
#include <vector>

#include <gtest/gtest.h>
#include <gtsam/config.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/NavState.h>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"
#include "mss/imu/preintegrator.hpp"

#include "../imu_test_utils.hpp"

namespace mss::imu {
namespace {

using common::ImuBias;
using common::ImuSample;
using common::Mat3;
using common::NavState;
using common::PreintegratedDelta;
using common::Vec3;
using test::default_imu_config;
using test::random_imu_sequence;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Mat93 = Eigen::Matrix<double, 9, 3>;

constexpr double kTol = 1e-9;  // I-03

// GTSAM params that make its 9x9 [phi, p, v] block equal to the derivation §4 recursion:
// continuous noise densities, no integration noise, no bias random walk or initial bias
// covariance (all four default to identity and would leak into the 9x9 block).
boost::shared_ptr<gtsam::PreintegrationCombinedParams> make_params(const ImuConfig& cfg) {
  auto p = gtsam::PreintegrationCombinedParams::MakeSharedU(cfg.gravity_mps2);
  p->accelerometerCovariance =
      cfg.acc_noise_density * cfg.acc_noise_density * gtsam::Matrix3::Identity();
  p->gyroscopeCovariance =
      cfg.gyro_noise_density * cfg.gyro_noise_density * gtsam::Matrix3::Identity();
  p->integrationCovariance = gtsam::Matrix3::Zero();
  p->biasAccCovariance = gtsam::Matrix3::Zero();
  p->biasOmegaCovariance = gtsam::Matrix3::Zero();
  p->biasAccOmegaInt = gtsam::Matrix6::Zero();
  return p;
}

gtsam::imuBias::ConstantBias to_gtsam(const ImuBias& b) {
  return gtsam::imuBias::ConstantBias(b.ba, b.bg);  // order: accel, gyro
}

ImuBias random_bias(std::mt19937& rng) {
  std::normal_distribution<double> nba(0.0, 0.1);   // m/s^2
  std::normal_distribution<double> nbg(0.0, 0.01);  // rad/s
  ImuBias b;
  for (int i = 0; i < 3; ++i) {
    b.ba(i) = nba(rng);
  }
  for (int i = 0; i < 3; ++i) {
    b.bg(i) = nbg(rng);
  }
  return b;
}

NavState make_x0(const std::vector<ImuSample>& seq, const ImuBias& bias) {
  NavState x0;
  x0.t = seq[0].t;
  x0.T_w_b.R = common::so3_exp(Vec3(0.1, 0.2, -0.3));
  x0.T_w_b.p = Vec3(1.0, 2.0, 3.0);
  x0.v_w_b = Vec3(1.0, -0.5, 0.2);
  x0.bias = bias;
  return x0;
}

ImuBias perturbed(const ImuBias& b) {
  ImuBias out = b;
  out.ba += Vec3(1e-3, -2e-3, 1.5e-3);
  out.bg += Vec3(1e-3, 5e-4, -1e-3);
  return out;
}

void expect_predict_matches(const Preintegrator& pre,
                            const gtsam::PreintegrationBase& ref,
                            const NavState& x0) {
  const NavState x1 = pre.predict(x0, common::kGravityW);
  const gtsam::NavState s0(gtsam::Rot3(x0.T_w_b.R), gtsam::Point3(x0.T_w_b.p), x0.v_w_b);
  const gtsam::NavState s1 = ref.predict(s0, to_gtsam(x0.bias));
  EXPECT_LT((x1.T_w_b.R - s1.attitude().matrix()).norm(), kTol);
  EXPECT_LT((x1.T_w_b.p - s1.position()).norm(), kTol);
  EXPECT_LT((x1.v_w_b - s1.velocity()).norm(), kTol);
}

TEST(ImuPreintGtsam, I03_MatchesManifoldPreintegration) {
  const ImuConfig cfg = default_imu_config();
  const std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  std::mt19937 rng(42);
  const ImuBias bias_lin = random_bias(rng);
  const auto params = make_params(cfg);

  Preintegrator pre(cfg);
  pre.reset(bias_lin);
  ASSERT_TRUE(pre.integrate(seq[0]).is_ok());
  gtsam::ManifoldPreintegration mp(params, to_gtsam(bias_lin));
  Mat9 sigma_ref = Mat9::Zero();
  const Mat3 a_cov = params->accelerometerCovariance;
  const Mat3 w_cov = params->gyroscopeCovariance;
  for (std::size_t k = 1; k < seq.size(); ++k) {
    ASSERT_TRUE(pre.integrate(seq[k]).is_ok());
    const double dt = common::seconds_between(seq[k - 1].t, seq[k].t);
    Mat9 A;
    Mat93 B;
    Mat93 C;
    mp.update(seq[k].acc_b, seq[k].gyro_b, dt, &A, &B, &C);
    // The 9x9 block of CombinedImuFactor.cpp's update with the zeroed params above.
    const Mat9 next = A * sigma_ref * A.transpose() + B * (a_cov / dt) * B.transpose() +
                      C * (w_cov / dt) * C.transpose();
    sigma_ref = next;
  }

  const PreintegratedDelta& d = pre.delta();
  EXPECT_LT((d.dR - mp.deltaRij().matrix()).norm(), kTol);
  EXPECT_LT((d.dp - mp.deltaPij()).norm(), kTol);
  EXPECT_LT((d.dv - mp.deltaVij()).norm(), kTol);
  EXPECT_NEAR(d.dt_s, mp.deltaTij(), kTol);
  EXPECT_LT((d.J_dR_bg - mp.delRdelBiasOmega()).norm(), kTol);
  EXPECT_LT((d.J_dp_ba - mp.delPdelBiasAcc()).norm(), kTol);
  EXPECT_LT((d.J_dp_bg - mp.delPdelBiasOmega()).norm(), kTol);
  EXPECT_LT((d.J_dv_ba - mp.delVdelBiasAcc()).norm(), kTol);
  EXPECT_LT((d.J_dv_bg - mp.delVdelBiasOmega()).norm(), kTol);
  EXPECT_LT((d.cov - sigma_ref).norm() / sigma_ref.norm(), kTol);  // relative (entries ~1e-8)

  expect_predict_matches(pre, mp, make_x0(seq, perturbed(bias_lin)));
}

TEST(ImuPreintGtsam, I03_MatchesCombinedMeasurements) {
  const ImuConfig cfg = default_imu_config();
  const auto params = make_params(cfg);
#ifndef GTSAM_TANGENT_PREINTEGRATION
  // Manifold build: PreintegratedCombinedMeasurements is ManifoldPreintegration; compare all.
  const std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  std::mt19937 rng(42);
  const ImuBias bias_lin = random_bias(rng);
#else
  // Tangent build (current pin): single-axis gyro, so Jr^-1(theta) w == w and theta adds up
  // exactly; only the quantities both schemes compute identically are compared.
  const std::vector<ImuSample> seq = random_imu_sequence(42, 201, /*single_axis_gyro=*/true);
  std::mt19937 rng(42);
  ImuBias bias_lin = random_bias(rng);
  bias_lin.bg = 0.01 * Vec3(1.0, 2.0, 3.0).normalized();
#endif
  gtsam::PreintegratedCombinedMeasurements pim(params, to_gtsam(bias_lin));
  for (std::size_t k = 1; k < seq.size(); ++k) {
    const double dt = common::seconds_between(seq[k - 1].t, seq[k].t);
    pim.integrateMeasurement(seq[k].acc_b, seq[k].gyro_b, dt);
  }
  Preintegrator pre(cfg);
  pre.reset(bias_lin);
  for (const ImuSample& s : seq) {
    ASSERT_TRUE(pre.integrate(s).is_ok());
  }
  const PreintegratedDelta& d = pre.delta();

  EXPECT_LT((d.dR - pim.deltaRij().matrix()).norm(), kTol);
  EXPECT_LT((d.dp - pim.deltaPij()).norm(), kTol);
  EXPECT_LT((d.dv - pim.deltaVij()).norm(), kTol);
  EXPECT_NEAR(d.dt_s, pim.deltaTij(), kTol);
#ifndef GTSAM_TANGENT_PREINTEGRATION
  EXPECT_LT((d.J_dR_bg - pim.delRdelBiasOmega()).norm(), kTol);
  EXPECT_LT((d.J_dp_ba - pim.delPdelBiasAcc()).norm(), kTol);
  EXPECT_LT((d.J_dp_bg - pim.delPdelBiasOmega()).norm(), kTol);
  EXPECT_LT((d.J_dv_ba - pim.delVdelBiasAcc()).norm(), kTol);
  EXPECT_LT((d.J_dv_bg - pim.delVdelBiasOmega()).norm(), kTol);
  const Mat9 cov_ref = pim.preintMeasCov().topLeftCorner<9, 9>();
  EXPECT_LT((d.cov - cov_ref).norm() / cov_ref.norm(), kTol);
  expect_predict_matches(pre, pim, make_x0(seq, perturbed(bias_lin)));
#else
  const Mat3 J_dp_ba_ref = pim.preintegrated_H_biasAcc().middleRows<3>(3);
  const Mat3 J_dv_ba_ref = pim.preintegrated_H_biasAcc().bottomRows<3>();
  EXPECT_LT((d.J_dp_ba - J_dp_ba_ref).norm(), kTol);
  EXPECT_LT((d.J_dv_ba - J_dv_ba_ref).norm(), kTol);
  RecordProperty("tangent_build",
                 "gyro-bias Jacobians and covariance checked against ManifoldPreintegration only");
#endif
}

}  // namespace
}  // namespace mss::imu
