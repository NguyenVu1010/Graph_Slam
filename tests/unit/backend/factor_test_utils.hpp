#pragma once

// Jacobian checks for GTSAM factors in GoogleTest (B-01).
// GTSAM's EXPECT_CORRECT_FACTOR_JACOBIANS is a CppUnitLite macro; its body is
// gtsam::internal::testFactorJacobians, wrapped here for gtest (approved by the controller).

#include <algorithm>
#include <cstddef>

#include <gtest/gtest.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

// Numerical step 1e-5 and absolute tolerance 1e-6 on the whitened Jacobians.
#define MSS_EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values)            \
  EXPECT_TRUE(gtsam::internal::testFactorJacobians(                    \
      ::testing::UnitTest::GetInstance()->current_test_info()->name(), \
      factor,                                                          \
      values,                                                          \
      1e-5,                                                            \
      1e-6))

namespace mss::backend::test {

// Largest relative Frobenius error, over the factor's keys, between the analytic
// (factor.linearize) and the central-difference (step 1e-5) whitened Jacobians.
inline double max_relative_jacobian_error(const gtsam::NoiseModelFactor& factor,
                                          const gtsam::Values& values) {
  const gtsam::JacobianFactor numerical = gtsam::linearizeNumerically(factor, values, 1e-5);
  const auto analytic =
      boost::dynamic_pointer_cast<gtsam::JacobianFactor>(factor.linearize(values));
  if (!analytic) {
    return 1.0;
  }
  double worst = 0.0;
  for (std::size_t k = 0; k < factor.size(); ++k) {
    const gtsam::Matrix A_num = numerical.getA(numerical.begin() + static_cast<long>(k));
    const gtsam::Matrix A_an = analytic->getA(analytic->begin() + static_cast<long>(k));
    const double denom = std::max(A_num.norm(), 1e-12);
    worst = std::max(worst, (A_an - A_num).norm() / denom);
  }
  return worst;
}

}  // namespace mss::backend::test
