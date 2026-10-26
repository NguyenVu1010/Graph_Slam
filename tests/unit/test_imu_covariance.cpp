// mss::imu::Preintegrator covariance (I-05). Chart and recursion:
// docs/derivations/imu_preintegration.md §1 and §4.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"
#include "mss/imu/preintegrator.hpp"

#include "imu_test_utils.hpp"

namespace mss::imu {
namespace {

using common::ImuBias;
using common::ImuSample;
using common::Mat3;
using common::PreintegratedDelta;
using common::Timestamp;
using common::Vec3;
using test::default_imu_config;
using test::random_imu_sequence;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Vec9 = Eigen::Matrix<double, 9, 1>;

PreintegratedDelta integrate_all(const ImuConfig& cfg, const std::vector<ImuSample>& seq) {
  Preintegrator pre(cfg);
  pre.reset(ImuBias{});
  for (const ImuSample& s : seq) {
    EXPECT_TRUE(pre.integrate(s).is_ok());
  }
  return pre.delta();
}

void expect_symmetric_psd(const Mat9& C) {
  EXPECT_TRUE(C.allFinite());
  EXPECT_LT((C - C.transpose()).norm() / C.norm(), 1e-12);
  const Eigen::SelfAdjointEigenSolver<Mat9> eig(C);
  EXPECT_GT(eig.eigenvalues().minCoeff(), -1e-15 * C.norm());
}

// Monte Carlo over 1000 noisy copies of one sequence (Test_CI_CD I-05: "within 10%").
// Made concrete (planner's call): per-block trace ratio within 10%, and mean NEES
// within 9 +/- 10%.
TEST(PreintegratorCov, I05_MatchesMonteCarlo) {
  const ImuConfig cfg = default_imu_config();
  const std::vector<ImuSample> nominal = random_imu_sequence(42, 201);
  const PreintegratedDelta d_nom = integrate_all(cfg, nominal);
  const Mat9 C = d_nom.cov;

  constexpr int kRuns = 1000;
  std::mt19937 rng(43);
  std::normal_distribution<double> unit(0.0, 1.0);
  std::vector<Vec9> errors(kRuns);
  std::vector<ImuSample> noisy = nominal;
  for (int r = 0; r < kRuns; ++r) {
    for (std::size_t k = 1; k < nominal.size(); ++k) {
      const double dt = common::seconds_between(nominal[k - 1].t, nominal[k].t);
      const double sa = cfg.acc_noise_density / std::sqrt(dt);   // m/s^2
      const double sg = cfg.gyro_noise_density / std::sqrt(dt);  // rad/s
      for (int i = 0; i < 3; ++i) {
        noisy[k].acc_b(i) = nominal[k].acc_b(i) + sa * unit(rng);
      }
      for (int i = 0; i < 3; ++i) {
        noisy[k].gyro_b(i) = nominal[k].gyro_b(i) + sg * unit(rng);
      }
    }
    const PreintegratedDelta d = integrate_all(cfg, noisy);
    const Mat3 dR_rel = d_nom.dR.transpose() * d.dR;
    Vec9 e;
    e.head<3>() = common::so3_log(dR_rel);
    e.segment<3>(3) = d_nom.dR.transpose() * (d.dp - d_nom.dp);
    e.tail<3>() = d_nom.dR.transpose() * (d.dv - d_nom.dv);
    errors[static_cast<std::size_t>(r)] = e;
  }

  Vec9 mean = Vec9::Zero();
  for (const Vec9& e : errors) {
    mean += e;
  }
  mean /= static_cast<double>(kRuns);
  Mat9 S = Mat9::Zero();
  for (const Vec9& e : errors) {
    const Vec9 c = e - mean;
    S += c * c.transpose();
  }
  S /= static_cast<double>(kRuns - 1);

  for (int b = 0; b < 3; ++b) {
    const double ratio = S.block<3, 3>(3 * b, 3 * b).trace() / C.block<3, 3>(3 * b, 3 * b).trace();
    EXPECT_LT(std::abs(ratio - 1.0), 0.10) << "block " << b << " (phi, p, v)";
  }

  const Eigen::LDLT<Mat9> ldlt(C);
  double nees = 0.0;
  for (const Vec9& e : errors) {
    const Vec9 Cinv_e = ldlt.solve(e);
    nees += e.dot(Cinv_e);
  }
  nees /= static_cast<double>(kRuns);
  EXPECT_GT(nees, 8.1);
  EXPECT_LT(nees, 9.9);
}

TEST(PreintegratorCov, SymmetricPsdAndZeroWithZeroNoise) {
  const std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  ImuConfig cfg = default_imu_config();
  expect_symmetric_psd(integrate_all(cfg, seq).cov);

  // Review Focus 2: one 15 ms gap step.
  std::vector<ImuSample> gap = {seq[0], seq[1]};
  gap[1].t = Timestamp{seq[0].t.ns + 15'000'000};
  expect_symmetric_psd(integrate_all(cfg, gap).cov);

  cfg.acc_noise_density = 0.0;
  cfg.gyro_noise_density = 0.0;
  EXPECT_TRUE(integrate_all(cfg, seq).cov == Mat9::Zero());
}

}  // namespace
}  // namespace mss::imu
