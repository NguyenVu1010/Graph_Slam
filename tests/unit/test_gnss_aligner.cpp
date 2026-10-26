#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/types.hpp"
#include "mss/gnss/gnss.hpp"

#include "alloc_counter.hpp"

namespace {

using mss::common::Mat3;
using mss::common::Pose;
using mss::common::Vec3;
using mss::gnss::GnssAligner;
using mss::gnss::GnssConfig;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kStepM = 0.2;  // m per fix (1 m/s at 5 Hz, ADR-011)
const double kNan = std::numeric_limits<double>::quiet_NaN();

double wrap_pi(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

Pose yaw_pose(double yaw_rad, const Vec3& p) {
  Pose T;
  T.R = Eigen::AngleAxisd(yaw_rad, Vec3::UnitZ()).toRotationMatrix();
  T.p = p;
  return T;
}

double yaw_of(const Mat3& R) {
  return std::atan2(R(1, 0), R(0, 0));
}

// Body pose k on a straight path in w with heading `heading_rad`; body yaw = heading.
Pose body_on_line(int k, double heading_rad) {
  const Vec3 dir(std::cos(heading_rad), std::sin(heading_rad), 0.0);
  return yaw_pose(heading_rad, Vec3(kStepM * k * dir));
}

GnssConfig cfg() {
  GnssConfig c;
  c.enabled = true;
  c.sigma_h_m = 1.0;
  c.sigma_v_m = 2.0;
  c.max_cov_m2 = 25.0;
  return c;
}

TEST(GnssAligner, ErrorSequenceOnStraightLine) {
  GnssAligner aligner(cfg());
  const Pose T_e_w = yaw_pose(0.0, Vec3::Zero());
  for (int n = 1; n <= 18; ++n) {
    const Pose T_w_b = body_on_line(n - 1, 0.0);
    const auto res = aligner.update(T_e_w * T_w_b.p, T_w_b, Vec3::Zero());
    if (n <= 9) {
      EXPECT_EQ(res.reason(), "not enough pairs") << "n " << n;
    } else if (n <= 17) {
      EXPECT_EQ(res.reason(), "insufficient horizontal spread") << "n " << n;
    } else {
      EXPECT_TRUE(res.is_ok()) << "n " << n << " " << res.reason();
    }
  }
}

TEST(GnssAligner, NonFiniteNotAdded) {
  GnssAligner aligner(cfg());
  int k = 0;
  auto good = [&]() {
    const Pose T_w_b = body_on_line(k++, 0.0);
    return aligner.update(T_w_b.p, T_w_b, Vec3::Zero());
  };
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(good().reason(), "not enough pairs");
  }
  const Pose T_ok = body_on_line(100, 0.0);
  Pose T_bad_r = T_ok;
  T_bad_r.R(0, 0) = kNan;
  Pose T_bad_p = T_ok;
  T_bad_p.p.y() = kNan;
  EXPECT_EQ(aligner.update(Vec3(kNan, 0.0, 0.0), T_ok, Vec3::Zero()).reason(), "non-finite input");
  EXPECT_EQ(aligner.update(T_ok.p, T_bad_r, Vec3::Zero()).reason(), "non-finite input");
  EXPECT_EQ(aligner.update(T_ok.p, T_bad_p, Vec3::Zero()).reason(), "non-finite input");
  EXPECT_EQ(aligner.update(T_ok.p, T_ok, Vec3(0.0, 0.0, kNan)).reason(), "non-finite input");
  EXPECT_EQ(good().reason(), "not enough pairs");                // 9th pair
  EXPECT_EQ(good().reason(), "insufficient horizontal spread");  // 10th pair
}

void check_exact(double yaw_true, const Vec3& p_true, double yaw_tol, double p_tol) {
  const Pose T_e_w = yaw_pose(yaw_true, p_true);
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  const double heading = 10.0 * kDeg;
  GnssAligner aligner(cfg());
  mss::common::Result<Pose> res = mss::common::Result<Pose>::error("none");
  for (int k = 0; k < 40; ++k) {
    const Pose T_w_b = body_on_line(k, heading);
    res = aligner.update(T_e_w * (T_w_b * p_b_g), T_w_b, p_b_g);
  }
  ASSERT_TRUE(res.is_ok()) << res.reason();
  const Pose& est = res.value();
  EXPECT_LT(std::abs(wrap_pi(yaw_of(est.R) - yaw_true)), yaw_tol);
  EXPECT_LT((est.p - p_true).norm(), p_tol);
  EXPECT_EQ(est.R(2, 2), 1.0);
  EXPECT_EQ(est.R(0, 2), 0.0);
}

TEST(GnssAligner, ExactNoiseFree) {
  check_exact(30.0 * kDeg, Vec3(120.0, -45.0, 3.0), 1e-9, 1e-9);
}

TEST(GnssAligner, LargeOffsetStaysExact) {  // Review Focus 3
  check_exact(30.0 * kDeg, Vec3(1.0e5, -2.0e5, 50.0), 1e-9, 1e-6);
}

TEST(GnssAligner, YawNearPiWraps) {  // Review Focus 4
  check_exact(179.9 * kDeg, Vec3(120.0, -45.0, 3.0), 1e-9, 1e-9);
  check_exact(-179.9 * kDeg, Vec3(120.0, -45.0, 3.0), 1e-9, 1e-9);
}

TEST(GnssAligner, StationaryJitterNeverAligns) {
  GnssAligner aligner(cfg());
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 1.0);
  const Pose T_w_b = yaw_pose(0.3, Vec3(5.0, 6.0, 0.0));
  for (int n = 1; n <= 200; ++n) {
    const Vec3 p_e_g = T_w_b.p + Vec3(noise(rng), noise(rng), noise(rng));
    const auto res = aligner.update(p_e_g, T_w_b, Vec3::Zero());
    if (n >= 10) {
      EXPECT_EQ(res.reason(), "insufficient horizontal spread") << "n " << n;
    }
  }
}

TEST(GnssAligner, NoAllocation) {
  GnssAligner aligner(cfg());
  const Pose T_e_w = yaw_pose(0.5, Vec3(10.0, 20.0, 1.0));
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  for (int k = 0; k < 20; ++k) {
    const Pose T_w_b = body_on_line(k, 0.2);
    (void)aligner.update(T_e_w * (T_w_b * p_b_g), T_w_b, p_b_g);
  }
  int n_ok = 0;
  AllocCounter::reset();
  {
    EigenMallocGuard guard;
    for (int k = 20; k < 120; ++k) {
      const Pose T_w_b = body_on_line(k, 0.2);
      n_ok += aligner.update(T_e_w * (T_w_b * p_b_g), T_w_b, p_b_g).is_ok() ? 1 : 0;
    }
    n_ok += aligner.update(Vec3(kNan, 0.0, 0.0), T_e_w, p_b_g).is_ok() ? 1 : 0;
  }
  EXPECT_EQ(AllocCounter::count(), 0u);
  EXPECT_EQ(n_ok, 100);
}

}  // namespace
