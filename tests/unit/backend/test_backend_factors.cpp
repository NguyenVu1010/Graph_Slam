// B-01: custom backend factors against numerical differentiation
// (docs/derivations/backend_factors.md sections 3, 5, 6, 7).
#include <cmath>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/preintegrator.hpp"
#include "mss/odom/odom_residual.hpp"

#include "conversions.hpp"
#include "factor_test_utils.hpp"
#include "gnss_factor.hpp"
#include "graph_scenarios.hpp"
#include "imu_factor.hpp"
#include "odom_factor.hpp"

namespace mss::backend::detail {
namespace {

using common::Mat3;
using common::Mat6;
using common::NavState;
using common::Pose;
using common::Vec3;
using common::Vec6;
using test::max_relative_jacobian_error;
using test::pose_from;
using test::random_pose;
using test::random_vec3;
using test::random_vec6;

constexpr std::uint32_t kSeed = 20261026;
constexpr double kRelJacTol = 1e-6;  // CLAUDE.md
const Vec3 kGravityW(0.0, 0.0, -9.80665);

common::ImuBias random_bias(std::mt19937& rng) {
  common::ImuBias b;
  b.ba = random_vec3(rng, 0.1);
  b.bg = random_vec3(rng, 0.01);
  return b;
}

common::PreintegratedDelta make_delta(std::mt19937& rng, const common::ImuBias& bias_lin) {
  const std::vector<common::ImuSample> seq =
      imu::test::random_imu_sequence(static_cast<std::uint32_t>(rng()), 41);
  imu::Preintegrator pre(imu::test::default_imu_config());
  pre.reset(bias_lin);
  for (const common::ImuSample& s : seq) {
    EXPECT_TRUE(pre.integrate(s).is_ok());
  }
  return pre.delta();
}

gtsam::Values imu_values(const NavState& x_i, const NavState& x_j, const common::ImuBias& b_i) {
  gtsam::Values v;
  v.insert(pose_key(0), to_gtsam(x_i.T_w_b));
  v.insert(vel_key(0), gtsam::Vector3(x_i.v_w_b));
  v.insert(pose_key(1), to_gtsam(x_j.T_w_b));
  v.insert(vel_key(1), gtsam::Vector3(x_j.v_w_b));
  v.insert(bias_key(0), to_gtsam(b_i));
  return v;
}

TEST(BackendFactors, B01_ImuFactorJacobians) {
  std::mt19937 rng(kSeed);
  for (int rep = 0; rep < 20; ++rep) {
    const common::ImuBias bias_lin = random_bias(rng);
    const common::PreintegratedDelta delta = make_delta(rng, bias_lin);
    NavState x_i;
    x_i.T_w_b = random_pose(rng);
    x_i.v_w_b = random_vec3(rng, 2.0);
    x_i.bias = bias_lin;
    // Prediction at bias_lin (imu_preintegration.md section 5), then perturbed.
    NavState x_j = x_i;
    const double T = delta.dt_s;
    x_j.T_w_b.R = x_i.T_w_b.R * delta.dR;
    x_j.v_w_b = x_i.v_w_b + kGravityW * T + x_i.T_w_b.R * delta.dv;
    x_j.T_w_b.p = x_i.T_w_b.p + x_i.v_w_b * T + 0.5 * kGravityW * T * T + x_i.T_w_b.R * delta.dp;
    x_j.T_w_b = x_j.T_w_b * common::se3_exp(random_vec6(rng, 0.05));
    x_j.v_w_b += random_vec3(rng, 0.1);
    common::ImuBias b_i = bias_lin;
    b_i.ba += random_vec3(rng, 0.01);
    b_i.bg += random_vec3(rng, 0.01);

    const ImuPreintFactor f(
        pose_key(0), vel_key(0), pose_key(1), vel_key(1), bias_key(0), delta, kGravityW);
    const gtsam::Values values = imu_values(x_i, x_j, b_i);
    ASSERT_GT(f.unwhitenedError(values).norm(), 1e-3);
    MSS_EXPECT_CORRECT_FACTOR_JACOBIANS(f, values);
    EXPECT_LT(max_relative_jacobian_error(f, values), kRelJacTol) << "rep " << rep;
  }
}

TEST(BackendFactors, B01_ImuFactorZeroAtPrediction) {
  std::mt19937 rng(kSeed);
  const common::ImuBias bias_lin = random_bias(rng);
  const std::vector<common::ImuSample> seq = imu::test::random_imu_sequence(kSeed, 41);
  imu::Preintegrator pre(imu::test::default_imu_config());
  pre.reset(bias_lin);
  for (const common::ImuSample& s : seq) {
    ASSERT_TRUE(pre.integrate(s).is_ok());
  }
  NavState x_i;
  x_i.t = seq.front().t;
  x_i.T_w_b = random_pose(rng);
  x_i.v_w_b = random_vec3(rng, 2.0);
  x_i.bias = bias_lin;
  x_i.bias.ba += Vec3(0.01, -0.02, 0.015);
  x_i.bias.bg += Vec3(0.002, 0.001, -0.003);
  const NavState x_j = pre.predict(x_i, kGravityW);
  const ImuPreintFactor f(
      pose_key(0), vel_key(0), pose_key(1), vel_key(1), bias_key(0), pre.delta(), kGravityW);
  const gtsam::Values values = imu_values(x_i, x_j, x_i.bias);
  EXPECT_LT(f.unwhitenedError(values).norm(), 1e-9);
}

struct GnssSetup {
  Pose T_w_b;
  Pose T_e_w = pose_from(Vec3(0.0, 0.0, 0.7), Vec3(100.0, -50.0, 3.0));
  Vec3 p_b_g = Vec3(0.3, -0.1, 0.8);
  Mat3 cov = Vec3(0.04, 0.09, 0.25).asDiagonal();
};

TEST(BackendFactors, B01_GnssFactorJacobians) {
  std::mt19937 rng(kSeed);
  for (int rep = 0; rep < 20; ++rep) {
    GnssSetup s;
    s.T_w_b = random_pose(rng);
    const Vec3 p_e_g = s.T_e_w * (s.T_w_b * s.p_b_g) + 0.5 * random_vec3(rng, 1.0).normalized();
    const GnssPositionFactor f(pose_key(0), p_e_g, s.cov, s.T_e_w, s.p_b_g);
    gtsam::Values values;
    values.insert(pose_key(0), to_gtsam(s.T_w_b));
    EXPECT_NEAR(f.unwhitenedError(values).norm(), 0.5, 1e-9);
    MSS_EXPECT_CORRECT_FACTOR_JACOBIANS(f, values);
    EXPECT_LT(max_relative_jacobian_error(f, values), kRelJacTol);
  }
}

TEST(BackendFactors, B01_GnssFactorZeroWhenConsistent) {
  std::mt19937 rng(kSeed);
  GnssSetup s;
  s.T_w_b = random_pose(rng);
  const Vec3 p_e_g = s.T_e_w * (s.T_w_b * s.p_b_g);
  const GnssPositionFactor f(pose_key(0), p_e_g, s.cov, s.T_e_w, s.p_b_g);
  gtsam::Values values;
  values.insert(pose_key(0), to_gtsam(s.T_w_b));
  EXPECT_LT(f.unwhitenedError(values).norm(), 1e-12);
}

TEST(BackendFactors, B01_OdomFactorRotatedSingularCov) {
  const Pose T_b_o = pose_from(Vec3(0.1, -0.2, 0.3), Vec3(0.2, 0.0, 0.1));
  const common::OdomDelta od = test::wheel_delta(T_b_o);
  const std::optional<OdomBetweenFactor> f =
      OdomBetweenFactor::create(pose_key(0), pose_key(1), od);
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->rank(), 3);
  EXPECT_EQ(f->dim(), 3U);

  // Pseudo-inverse of the covariance from the same eigen split.
  const Eigen::SelfAdjointEigenSolver<Mat6> es(od.cov);
  const double lambda_max = es.eigenvalues()(5);
  Mat6 pinv = Mat6::Zero();
  for (int k = 0; k < 6; ++k) {
    const double l = es.eigenvalues()(k);
    if (l > kOdomRankRelTol * lambda_max) {
      pinv += es.eigenvectors().col(k) * es.eigenvectors().col(k).transpose() / l;
    }
  }

  std::mt19937 rng(kSeed);
  for (int rep = 0; rep < 20; ++rep) {
    const Pose T_i = random_pose(rng);
    const Pose T_j = T_i * od.T_b0_b1 * common::se3_exp(random_vec6(rng, 0.05));
    gtsam::Values values;
    values.insert(pose_key(0), to_gtsam(T_i));
    values.insert(pose_key(1), to_gtsam(T_j));
    MSS_EXPECT_CORRECT_FACTOR_JACOBIANS(*f, values);
    EXPECT_LT(max_relative_jacobian_error(*f, values), kRelJacTol);
    const Vec6 r_m = odom::relative_pose_residual(T_i, T_j, od.T_b0_b1).r;
    const double expected = r_m.dot(pinv * r_m);
    EXPECT_NEAR(2.0 * f->error(values), expected, 1e-9 * expected);
  }
}

TEST(BackendFactors, B01_OdomFactorZeroCovRejected) {
  common::OdomDelta od = test::wheel_delta(Pose{});
  od.cov.setZero();
  EXPECT_FALSE(OdomBetweenFactor::create(pose_key(0), pose_key(1), od).has_value());
  od = test::wheel_delta(Pose{});
  od.cov(2, 2) = std::nan("");
  EXPECT_FALSE(OdomBetweenFactor::create(pose_key(0), pose_key(1), od).has_value());
}

// Not a custom factor: checks the permuted noise path and the Cauchy wrap used for loops.
TEST(BackendFactors, B01_BetweenAndRobustLoopJacobians) {
  std::mt19937 rng(kSeed);
  Mat6 S_m = Mat6::Zero();
  S_m.diagonal() << 0.04, 0.09, 0.01, 0.0025, 0.0004, 0.0009;  // [rho, phi]
  const Mat6 L = S_m.llt().matrixL();
  const Pose T_i = random_pose(rng);
  const Pose Z = random_pose(rng);
  const auto gaussian = gtsam::noiseModel::Gaussian::Covariance(permute_cov(S_m));
  const auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(3.5485), gaussian);

  // T_j such that the mss residual is L * u with |u| = the whitened norm.
  const auto values_at = [&](double whitened_norm) {
    const Vec6 u = whitened_norm * random_vec6(rng, 1.0).normalized();
    const Vec6 r_m = L * u;
    const Pose T_j = T_i * Z * common::se3_exp(r_m);
    gtsam::Values v;
    v.insert(pose_key(0), to_gtsam(T_i));
    v.insert(pose_key(1), to_gtsam(T_j));
    return v;
  };

  const gtsam::BetweenFactor<gtsam::Pose3> f_gauss(pose_key(0), pose_key(1), to_gtsam(Z), gaussian);
  const gtsam::BetweenFactor<gtsam::Pose3> f_robust(pose_key(0), pose_key(1), to_gtsam(Z), robust);

  // gtsam::BetweenFactor differentiates only Between(), not the Logmap of the error, so its
  // Jacobians are exact at zero error only (GTSAM factorTesting.h note). The permuted
  // noise and the Cauchy wrap are therefore checked at zero error, where the numerical
  // Jacobians of the whitened (and reweighted) error are exact.
  const gtsam::Values v0 = values_at(0.0);
  EXPECT_LT(f_gauss.whitenedError(v0).norm(), 1e-12);
  MSS_EXPECT_CORRECT_FACTOR_JACOBIANS(f_gauss, v0);
  EXPECT_LT(max_relative_jacobian_error(f_gauss, v0), kRelJacTol);
  MSS_EXPECT_CORRECT_FACTOR_JACOBIANS(f_robust, v0);
  EXPECT_LT(max_relative_jacobian_error(f_robust, v0), kRelJacTol);

  const gtsam::Values v01 = values_at(0.1);
  EXPECT_NEAR(f_gauss.whitenedError(v01).norm(), 0.1, 1e-9);
  // At |e| = 0.1 the robust linearisation is the Gaussian one scaled by sqrt(w).
  const double w = 1.0 / (1.0 + (0.1 / 3.5485) * (0.1 / 3.5485));
  const auto lin_g = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(f_gauss.linearize(v01));
  const auto lin_r = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(f_robust.linearize(v01));
  ASSERT_TRUE(lin_g && lin_r);
  for (int k = 0; k < 2; ++k) {
    const gtsam::Matrix A_g = lin_g->getA(lin_g->begin() + k);
    const gtsam::Matrix A_r = lin_r->getA(lin_r->begin() + k);
    EXPECT_LT((A_r - std::sqrt(w) * A_g).norm(), 1e-9 * A_g.norm());
  }
}

}  // namespace
}  // namespace mss::backend::detail
