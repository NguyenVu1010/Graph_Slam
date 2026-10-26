// mss::imu::Preintegrator: mean, bias Jacobians and predict (I-01, I-02, I-04).
// Formulas: docs/derivations/imu_preintegration.md.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"
#include "mss/imu/preintegrator.hpp"

#include "alloc_counter.hpp"
#include "imu_test_utils.hpp"

namespace mss::imu {
namespace {

using common::ImuBias;
using common::ImuSample;
using common::Mat3;
using common::NavState;
using common::PreintegratedDelta;
using common::Timestamp;
using common::Vec3;
using test::default_imu_config;
using test::random_imu_sequence;

constexpr double kGravityMps2 = 9.80665;

// Integrates seq[first, last] (inclusive) after reset(bias); every sample must be accepted.
void integrate_range(Preintegrator& pre,
                     const std::vector<ImuSample>& seq,
                     std::size_t first,
                     std::size_t last,
                     const ImuBias& bias) {
  pre.reset(bias);
  for (std::size_t k = first; k <= last; ++k) {
    ASSERT_TRUE(pre.integrate(seq[k]).is_ok()) << "sample " << k;
  }
}

void expect_delta_eq(const PreintegratedDelta& a, const PreintegratedDelta& b) {
  EXPECT_TRUE(a.dR == b.dR);
  EXPECT_TRUE(a.dv == b.dv);
  EXPECT_TRUE(a.dp == b.dp);
  EXPECT_EQ(a.dt_s, b.dt_s);
  EXPECT_TRUE(a.cov == b.cov);
  EXPECT_TRUE(a.J_dR_bg == b.J_dR_bg);
  EXPECT_TRUE(a.J_dv_ba == b.J_dv_ba);
  EXPECT_TRUE(a.J_dv_bg == b.J_dv_bg);
  EXPECT_TRUE(a.J_dp_ba == b.J_dp_ba);
  EXPECT_TRUE(a.J_dp_bg == b.J_dp_bg);
  EXPECT_TRUE(a.bias_lin.bg == b.bias_lin.bg);
  EXPECT_TRUE(a.bias_lin.ba == b.bias_lin.ba);
}

// ba ~ N(0, 0.1) m/s^2, then bg ~ N(0, 0.01) rad/s per axis.
ImuBias random_bias(std::mt19937& rng) {
  std::normal_distribution<double> nba(0.0, 0.1);
  std::normal_distribution<double> nbg(0.0, 0.01);
  ImuBias b;
  for (int i = 0; i < 3; ++i) {
    b.ba(i) = nba(rng);
  }
  for (int i = 0; i < 3; ++i) {
    b.bg(i) = nbg(rng);
  }
  return b;
}

NavState reference_x0(Timestamp t, const ImuBias& bias) {
  NavState x0;
  x0.t = t;
  x0.T_w_b.R = common::so3_exp(Vec3(0.1, 0.2, -0.3));
  x0.T_w_b.p = Vec3(1.0, 2.0, 3.0);
  x0.v_w_b = Vec3(1.0, -0.5, 0.2);
  x0.bias = bias;
  return x0;
}

TEST(Preintegrator, CtorRejectsInvalidConfig) {
  ImuConfig cfg = default_imu_config();
  cfg.acc_noise_density = -1e-3;
  EXPECT_THROW(Preintegrator{cfg}, std::invalid_argument);
  cfg = default_imu_config();
  cfg.gyro_noise_density = -1e-3;
  EXPECT_THROW(Preintegrator{cfg}, std::invalid_argument);
  cfg = default_imu_config();
  cfg.rate_hz = 0.0;
  EXPECT_THROW(Preintegrator{cfg}, std::invalid_argument);
  cfg.rate_hz = -200.0;
  EXPECT_THROW(Preintegrator{cfg}, std::invalid_argument);
  cfg = default_imu_config();
  cfg.acc_noise_density = 0.0;
  cfg.gyro_noise_density = 0.0;
  EXPECT_NO_THROW(Preintegrator{cfg});
}

TEST(Preintegrator, I01_StaticGravityOnlyPredictsNoMotion) {
  const Mat3 R0 = common::so3_exp(Vec3(0.3, -0.2, 1.0));
  std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  for (ImuSample& s : seq) {
    s.acc_b = R0.transpose() * Vec3(0.0, 0.0, kGravityMps2);
    s.gyro_b = Vec3::Zero();
  }
  Preintegrator pre(default_imu_config());
  integrate_range(pre, seq, 0, 200, ImuBias{});
  EXPECT_LT((pre.delta().dR - Mat3::Identity()).norm(), 1e-9);

  NavState x0;
  x0.t = seq[0].t;
  x0.T_w_b.R = R0;
  x0.T_w_b.p = Vec3(1.0, 2.0, 3.0);
  const NavState x1 = pre.predict(x0, common::kGravityW);
  EXPECT_LT((x1.T_w_b.R - R0).norm(), 1e-9);
  EXPECT_LT((x1.T_w_b.p - x0.T_w_b.p).norm(), 1e-9);
  EXPECT_LT(x1.v_w_b.norm(), 1e-9);
  EXPECT_EQ(x1.t.ns, seq[200].t.ns);
}

TEST(Preintegrator, I02_ConstantRateMatchesExp) {
  const Vec3 w(0.3, -0.2, 0.5);
  ImuBias bias;
  bias.bg = Vec3(0.01, -0.02, 0.005);
  std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  for (ImuSample& s : seq) {
    s.gyro_b = w + bias.bg;
  }
  Preintegrator pre(default_imu_config());
  integrate_range(pre, seq, 0, 200, bias);
  const Mat3 expected = common::so3_exp(Vec3(w * pre.delta().dt_s));
  EXPECT_LT((pre.delta().dR - expected).norm(), 1e-9);
}

TEST(Preintegrator, PredictWithNoSampleReturnsInput) {
  const std::vector<ImuSample> seq = random_imu_sequence(42, 3);
  Preintegrator pre(default_imu_config());
  const NavState x0 = reference_x0(seq[0].t, ImuBias{});
  auto expect_same = [&x0](const NavState& x) {
    EXPECT_EQ(x.t, x0.t);
    EXPECT_TRUE(x.T_w_b.R == x0.T_w_b.R);
    EXPECT_TRUE(x.T_w_b.p == x0.T_w_b.p);
    EXPECT_TRUE(x.v_w_b == x0.v_w_b);
    EXPECT_TRUE(x.bias.bg == x0.bias.bg);
    EXPECT_TRUE(x.bias.ba == x0.bias.ba);
  };
  pre.reset(ImuBias{});
  expect_same(pre.predict(x0, common::kGravityW));
  ASSERT_TRUE(pre.integrate(seq[0]).is_ok());
  EXPECT_EQ(pre.delta().dt_s, 0.0);
  expect_same(pre.predict(x0, common::kGravityW));
}

// Residuals: dR (right perturbation, so3_log(dR0^T dR)), dv and dp (additive), as
// functions of the linearisation bias. Variables: ba (m/s^2), bg (rad/s).
// Derivation §3; central differences with step 1e-5, relative error < 1e-6.
TEST(Preintegrator, I04_BiasJacobiansMatchCentralDifferences) {
  const std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  std::mt19937 rng(42);
  const ImuBias bias_lin = random_bias(rng);
  constexpr double kH = 1e-5;

  Preintegrator pre(default_imu_config());
  integrate_range(pre, seq, 0, 200, bias_lin);
  const PreintegratedDelta d0 = pre.delta();

  Mat3 num_dR_ba;
  Mat3 num_dR_bg;
  Mat3 num_dv_ba;
  Mat3 num_dv_bg;
  Mat3 num_dp_ba;
  Mat3 num_dp_bg;
  for (int which = 0; which < 2; ++which) {  // 0: ba, 1: bg
    for (int i = 0; i < 3; ++i) {
      ImuBias bp = bias_lin;
      ImuBias bm = bias_lin;
      (which == 0 ? bp.ba : bp.bg)(i) += kH;
      (which == 0 ? bm.ba : bm.bg)(i) -= kH;
      Preintegrator pp(default_imu_config());
      Preintegrator pm(default_imu_config());
      integrate_range(pp, seq, 0, 200, bp);
      integrate_range(pm, seq, 0, 200, bm);
      const Mat3 dR_rel_p = d0.dR.transpose() * pp.delta().dR;
      const Mat3 dR_rel_m = d0.dR.transpose() * pm.delta().dR;
      const Vec3 col_R = (common::so3_log(dR_rel_p) - common::so3_log(dR_rel_m)) / (2.0 * kH);
      const Vec3 col_v = (pp.delta().dv - pm.delta().dv) / (2.0 * kH);
      const Vec3 col_p = (pp.delta().dp - pm.delta().dp) / (2.0 * kH);
      (which == 0 ? num_dR_ba : num_dR_bg).col(i) = col_R;
      (which == 0 ? num_dv_ba : num_dv_bg).col(i) = col_v;
      (which == 0 ? num_dp_ba : num_dp_bg).col(i) = col_p;
    }
  }
  EXPECT_LT((num_dR_bg - d0.J_dR_bg).norm() / d0.J_dR_bg.norm(), 1e-6);
  EXPECT_LT((num_dv_ba - d0.J_dv_ba).norm() / d0.J_dv_ba.norm(), 1e-6);
  EXPECT_LT((num_dv_bg - d0.J_dv_bg).norm() / d0.J_dv_bg.norm(), 1e-6);
  EXPECT_LT((num_dp_ba - d0.J_dp_ba).norm() / d0.J_dp_ba.norm(), 1e-6);
  EXPECT_LT((num_dp_bg - d0.J_dp_bg).norm() / d0.J_dp_bg.norm(), 1e-6);
  EXPECT_LT(num_dR_ba.norm(), 1e-9);  // dR does not depend on ba
}

TEST(Preintegrator, I04_PredictBiasCorrectionIsFirstOrder) {
  const std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  std::mt19937 rng(42);
  const ImuBias bias_lin = random_bias(rng);
  ImuBias bias_true = bias_lin;
  bias_true.ba += Vec3(1e-3, -2e-3, 1.5e-3);
  bias_true.bg += Vec3(1e-3, 5e-4, -1e-3);

  Preintegrator pre_lin(default_imu_config());
  Preintegrator pre_true(default_imu_config());
  integrate_range(pre_lin, seq, 0, 200, bias_lin);
  integrate_range(pre_true, seq, 0, 200, bias_true);

  const NavState x0 = reference_x0(seq[0].t, bias_true);
  const NavState xa = pre_lin.predict(x0, common::kGravityW);
  const NavState xb = pre_true.predict(x0, common::kGravityW);
  const Mat3 dR_ab = xa.T_w_b.R.transpose() * xb.T_w_b.R;
  EXPECT_LT(common::so3_log(dR_ab).norm(), 1e-5);
  EXPECT_LT((xa.T_w_b.p - xb.T_w_b.p).norm(), 1e-5);
  EXPECT_LT((xa.v_w_b - xb.v_w_b).norm(), 1e-5);
  EXPECT_TRUE(xa.bias.ba == x0.bias.ba);
  EXPECT_TRUE(xa.bias.bg == x0.bias.bg);
}

// Review Focus 1: out-of-order, duplicate or non-finite samples mid-interval.
TEST(Preintegrator, RejectedSampleLeavesDeltaUntouchedAndStreamContinues) {
  const std::vector<ImuSample> seq = random_imu_sequence(42, 60);
  Preintegrator pre(default_imu_config());
  integrate_range(pre, seq, 0, 49, ImuBias{});
  const PreintegratedDelta snapshot = pre.delta();

  ImuSample dup = seq[50];
  dup.t = seq[49].t;
  EXPECT_EQ(pre.integrate(dup).reason(), "imu timestamp not increasing");
  expect_delta_eq(pre.delta(), snapshot);

  ImuSample older = seq[50];
  older.t = seq[48].t;
  EXPECT_EQ(pre.integrate(older).reason(), "imu timestamp not increasing");
  expect_delta_eq(pre.delta(), snapshot);

  ImuSample nan_acc = seq[50];
  nan_acc.acc_b.y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(pre.integrate(nan_acc).reason(), "non-finite imu sample");
  expect_delta_eq(pre.delta(), snapshot);

  ImuSample inf_gyro = seq[50];
  inf_gyro.gyro_b.z() = std::numeric_limits<double>::infinity();
  EXPECT_EQ(pre.integrate(inf_gyro).reason(), "non-finite imu sample");
  expect_delta_eq(pre.delta(), snapshot);

  ASSERT_TRUE(pre.integrate(seq[50]).is_ok());
  Preintegrator ref(default_imu_config());
  integrate_range(ref, seq, 0, 50, ImuBias{});
  expect_delta_eq(pre.delta(), ref.delta());
}

// Review Focus 2: a dropped-sample gap integrates like the subdivided steps when w = 0.
TEST(Preintegrator, GapEqualsSubdividedStepsWithoutRotation) {
  const Vec3 acc(0.4, -0.3, 9.9);
  auto sample = [&acc](std::int64_t t_ns) {
    ImuSample s;
    s.t = Timestamp{t_ns};
    s.acc_b = acc;
    return s;
  };
  constexpr std::int64_t kT0Ns = 1'000'000'000;
  constexpr std::int64_t kStepNs = 5'000'000;
  Preintegrator a(default_imu_config());
  Preintegrator b(default_imu_config());
  ASSERT_TRUE(a.integrate(sample(kT0Ns)).is_ok());
  ASSERT_TRUE(a.integrate(sample(kT0Ns + 3 * kStepNs)).is_ok());
  for (std::int64_t k = 0; k <= 3; ++k) {
    ASSERT_TRUE(b.integrate(sample(kT0Ns + k * kStepNs)).is_ok());
  }
  EXPECT_LT((a.delta().dv - b.delta().dv).norm(), 1e-12);
  EXPECT_LT((a.delta().dp - b.delta().dp).norm(), 1e-12);
  EXPECT_EQ(a.delta().dt_s, b.delta().dt_s);
}

// Review Focus 3: a non-finite first sample must not start the interval.
TEST(Preintegrator, NonFiniteFirstSampleDoesNotStartInterval) {
  const std::vector<ImuSample> seq = random_imu_sequence(42, 3);
  Preintegrator pre(default_imu_config());
  pre.reset(ImuBias{});
  ImuSample bad = seq[0];
  bad.acc_b.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(pre.integrate(bad).is_ok());
  ASSERT_TRUE(pre.integrate(seq[1]).is_ok());
  EXPECT_EQ(pre.delta().dt_s, 0.0);
  ASSERT_TRUE(pre.integrate(seq[2]).is_ok());
  EXPECT_EQ(pre.delta().dt_s, common::seconds_between(seq[1].t, seq[2].t));
  const NavState x0 = reference_x0(seq[1].t, ImuBias{});
  EXPECT_EQ(pre.predict(x0, common::kGravityW).t, seq[2].t);
}

// Review Focus 4: keyframe chaining (the last sample of an interval is re-fed).
TEST(Preintegrator, ChainedDeltasEqualSingleDelta) {
  const std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  std::mt19937 rng(42);
  const ImuBias bias_lin = random_bias(rng);
  Preintegrator p1(default_imu_config());
  Preintegrator p2(default_imu_config());
  Preintegrator pf(default_imu_config());
  integrate_range(p1, seq, 0, 100, bias_lin);
  integrate_range(p2, seq, 100, 200, bias_lin);
  integrate_range(pf, seq, 0, 200, bias_lin);

  const NavState x0 = reference_x0(seq[0].t, bias_lin);
  const NavState xa = p2.predict(p1.predict(x0, common::kGravityW), common::kGravityW);
  const NavState xb = pf.predict(x0, common::kGravityW);
  EXPECT_LT((xa.T_w_b.R - xb.T_w_b.R).norm(), 1e-9);
  EXPECT_LT((xa.T_w_b.p - xb.T_w_b.p).norm(), 1e-9);
  EXPECT_LT((xa.v_w_b - xb.v_w_b).norm(), 1e-9);
  EXPECT_EQ(xa.t, xb.t);
}

TEST(Preintegrator, HotPathDoesNotAllocate) {
  const std::vector<ImuSample> seq = random_imu_sequence(42, 201);
  Preintegrator pre(default_imu_config());
  const NavState x0 = reference_x0(seq[0].t, ImuBias{});
  ImuSample nan_sample = seq[200];
  nan_sample.acc_b.x() = std::numeric_limits<double>::quiet_NaN();
  const ImuSample stale = seq[100];
  ImuBias bias;
  bias.bg = Vec3(0.01, 0.0, 0.0);
  NavState x1;
  bool all_ok = true;
  bool rejected = true;

  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    pre.reset(bias);
    for (std::size_t k = 0; k < 200; ++k) {
      all_ok = pre.integrate(seq[k]).is_ok() && all_ok;
    }
    x1 = pre.predict(x0, common::kGravityW);
    rejected = !pre.integrate(nan_sample).is_ok() && rejected;
    rejected = !pre.integrate(stale).is_ok() && rejected;
    pre.reset(bias);
    all_ok = pre.integrate(seq[0]).is_ok() && all_ok;
    all_ok = pre.integrate(seq[1]).is_ok() && all_ok;
  }
  EXPECT_EQ(AllocCounter::count(), 0u);
  EXPECT_TRUE(all_ok);
  EXPECT_TRUE(rejected);
  EXPECT_TRUE(x1.T_w_b.p.allFinite());
}

}  // namespace
}  // namespace mss::imu
