// core/backend conversions (docs/derivations/backend_factors.md section 1).
#include <cmath>
#include <random>

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"

#include "conversions.hpp"

namespace mss::backend::detail {
namespace {

using common::Mat6;
using common::Pose;
using common::Vec6;

Vec6 random_vec6(std::mt19937& rng, double sigma) {
  std::normal_distribution<double> n(0.0, sigma);
  Vec6 v;
  for (int i = 0; i < 6; ++i) {
    v(i) = n(rng);
  }
  return v;
}

Mat6 random_spd(std::mt19937& rng) {
  Mat6 A;
  std::normal_distribution<double> n(0.0, 1.0);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      A(r, c) = n(rng);
    }
  }
  const Mat6 S = 1e-3 * (A * A.transpose() + Mat6::Identity());
  return S;
}

TEST(BackendConversions, PoseRoundTrip) {
  std::mt19937 rng(42);
  for (int k = 0; k < 1000; ++k) {
    const Pose T = common::se3_exp(random_vec6(rng, 2.0));
    const Pose back = from_gtsam(to_gtsam(T));
    EXPECT_LT((back.R - T.R).norm(), 1e-12);
    EXPECT_LT((back.p - T.p).norm(), 1e-12);
  }
}

TEST(BackendConversions, PermuteCovIsInvolution) {
  std::mt19937 rng(42);
  const Mat6 S = random_spd(rng);
  const Mat6 P = permutation_matrix();
  const Mat6 expected = P * S * P.transpose();
  EXPECT_EQ(permute_cov(S), expected);
  EXPECT_EQ(permute_cov(permute_cov(S)), S);
  EXPECT_EQ(P * P, Mat6::Identity());
}

// Whitened squared error of a GTSAM BetweenFactor with the permuted covariance equals the
// mss Mahalanobis norm of the mss residual.
TEST(BackendConversions, CovariancePermutationMatchesGtsamWhitening) {
  std::mt19937 rng(42);
  for (int k = 0; k < 20; ++k) {
    const Mat6 sigma_m = random_spd(rng);
    const Pose T0 = common::se3_exp(random_vec6(rng, 1.0));
    const Pose T1 = common::se3_exp(random_vec6(rng, 1.0));
    const Pose Z = (T0.inverse() * T1) * common::se3_exp(random_vec6(rng, 0.1));
    const gtsam::BetweenFactor<gtsam::Pose3> f(
        pose_key(0),
        pose_key(1),
        to_gtsam(Z),
        gtsam::noiseModel::Gaussian::Covariance(permute_cov(sigma_m)));
    gtsam::Values values;
    values.insert(pose_key(0), to_gtsam(T0));
    values.insert(pose_key(1), to_gtsam(T1));
    const Vec6 r_m = common::se3_log(Z.inverse() * T0.inverse() * T1);
    const double expected = r_m.dot(sigma_m.ldlt().solve(r_m));
    const double actual = 2.0 * f.error(values);
    EXPECT_NEAR(actual, expected, 1e-10 * expected);
  }
}

TEST(BackendConversions, KeysAreDistinctSymbols) {
  EXPECT_EQ(pose_key(3), gtsam::Symbol('x', 3).key());
  EXPECT_EQ(vel_key(3), gtsam::Symbol('v', 3).key());
  EXPECT_EQ(bias_key(3), gtsam::Symbol('b', 3).key());
}

}  // namespace
}  // namespace mss::backend::detail
