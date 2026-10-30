#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <string>

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
  c.align_check_yaw_sigma = false;  // geometry tests; the check has its own tests below (G13)
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

TEST(GnssAligner, FrozenFixWhileMovingNeverAligns) {
  // Review I-1: the receiver repeats one fix while the body drives 40 m. The w side has
  // plenty of spread, the e side none: yaw is unobservable, so no pose may be returned.
  GnssAligner aligner(cfg());
  const Vec3 p_e_g_frozen(120.0, -45.0, 3.0);
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  for (int n = 1; n <= 200; ++n) {
    const Pose T_w_b = body_on_line(n - 1, 10.0 * kDeg);
    const auto res = aligner.update(p_e_g_frozen, T_w_b, p_b_g);
    if (n < GnssAligner::kMinPairsForAlignment) {
      EXPECT_EQ(res.reason(), "not enough pairs") << "n " << n;
    } else {
      EXPECT_EQ(res.reason(), "insufficient horizontal spread") << "n " << n;
    }
  }
}

// Body on a 5 m circle (yaw tangent), antenna lever arm applied; noise-free pairs.
Pose body_on_circle(int k) {
  const double a = 0.5 * k;
  return yaw_pose(a + 0.5 * kPi, Vec3(5.0 * std::cos(a), 5.0 * std::sin(a), 0.0));
}

TEST(GnssAligner, UsesConfiguredMinPairs) {
  GnssConfig c = cfg();
  c.align_min_pairs = 3;
  GnssAligner aligner(c);
  const Pose T_e_w = yaw_pose(30.0 * kDeg, Vec3(5.0, -3.0, 1.0));
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  for (int n = 1; n <= 3; ++n) {
    const Pose T_w_b = body_on_circle(n);
    const auto res = aligner.update(T_e_w * (T_w_b * p_b_g), T_w_b, p_b_g);
    if (n < 3) {
      EXPECT_EQ(res.reason(), "not enough pairs") << "n " << n;
    } else {
      ASSERT_TRUE(res.is_ok()) << res.reason();
      EXPECT_NEAR(wrap_pi(yaw_of(res.value().R) - 30.0 * kDeg), 0.0, 1e-9);
    }
  }
}

TEST(GnssAligner, UsesConfiguredSpread) {
  GnssConfig c = cfg();
  c.align_min_spread_m = 10.0;
  GnssAligner aligner(c);
  const Pose T_e_w = yaw_pose(30.0 * kDeg, Vec3(5.0, -3.0, 1.0));
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  std::string last;
  for (int n = 1; n <= 12; ++n) {
    const Pose T_w_b = body_on_circle(n);
    last = aligner.update(T_e_w * (T_w_b * p_b_g), T_w_b, p_b_g).reason();
  }
  EXPECT_EQ(last, "insufficient horizontal spread");
}

// PR 4a (gnss_fusion_study.md section 2): T_e_w is re-estimated from all pairs at every
// call after alignment, it is not frozen at the first estimate.
TEST(GnssAligner, ReestimatesAsPairsArrive) {
  GnssAligner aligner(cfg());
  const Pose T_e_w = yaw_pose(30.0 * kDeg, Vec3(5.0, -3.0, 1.0));
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.3);  // m
  Pose first;
  Pose last;
  int n_ok = 0;
  for (int n = 1; n <= 200; ++n) {
    const Pose T_w_b = body_on_circle(n);
    const Vec3 p_e_g = T_e_w * (T_w_b * p_b_g) + Vec3(noise(rng), noise(rng), noise(rng));
    const auto res = aligner.update(p_e_g, T_w_b, p_b_g);
    if (res.is_ok()) {
      if (n_ok == 0) {
        first = res.value();
      }
      last = res.value();
      ++n_ok;
    }
  }
  ASSERT_EQ(n_ok, 191);  // from the 10th pair on
  EXPECT_GT((first.p - last.p).norm() + std::abs(wrap_pi(yaw_of(first.R) - yaw_of(last.R))), 1e-6);
  // 200 pairs, spread 5 m, sigma 0.3 m: yaw 1-sigma 0.3 / (sqrt(200) 5) = 0.24 deg.
  EXPECT_LT(std::abs(wrap_pi(yaw_of(last.R) - 30.0 * kDeg)), 1.0 * kDeg);
  EXPECT_LT((last.p - T_e_w.p).head<2>().norm(), 0.15);
}

// Backlog G1: the predicted yaw sigma, gnss.md section 4. Four pairs at the corners of a
// diamond of radius 1 m: sum |a_c|^2 = 4 m^2, so sigma = sqrt(4 var) / 4 = sqrt(var) / 2.
TEST(GnssAligner, YawSigmaKnownValue) {
  GnssConfig c = cfg();
  c.align_min_pairs = 4;
  c.align_yaw_min_var_m2 = 0.25;
  const Vec3 corners[4] = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(-1, 0, 0), Vec3(0, -1, 0)};
  const auto sigma_with = [&](const Mat3& cov, bool three_arg) {
    GnssAligner aligner(c);
    EXPECT_TRUE(std::isnan(aligner.yaw_sigma_rad()));
    mss::common::Result<Pose> res = mss::common::Result<Pose>::error("none");
    for (const Vec3& p : corners) {
      const Pose T_w_b = yaw_pose(0.0, p);
      res = three_arg ? aligner.update(p, T_w_b, Vec3::Zero())
                      : aligner.update(p, cov, T_w_b, Vec3::Zero());
    }
    EXPECT_TRUE(res.is_ok()) << res.reason();
    return aligner.yaw_sigma_rad();
  };
  const Mat3 iso = Mat3::Identity();
  EXPECT_NEAR(sigma_with(iso, false), 0.5, 1e-12);
  // Anisotropic: the larger horizontal eigenvalue (4 m^2) counts; the vertical is ignored.
  Mat3 aniso = Mat3::Identity();
  aniso(0, 0) = 4.0;
  aniso(2, 2) = 100.0;
  EXPECT_NEAR(sigma_with(aniso, false), 1.0, 1e-12);
  // Off-diagonal: [[1, 1], [1, 1]] has eigenvalues 0 and 2.
  Mat3 corr = Mat3::Identity();
  corr(0, 1) = 1.0;
  corr(1, 0) = 1.0;
  EXPECT_NEAR(sigma_with(corr, false), std::sqrt(2.0) / 2.0, 1e-12);
  // Below the floor (here negative): 0.25 m^2; the 3-argument update uses the floor too.
  const Mat3 neg = -Mat3::Identity();
  EXPECT_NEAR(sigma_with(neg, false), 0.25, 1e-12);
  EXPECT_NEAR(sigma_with(Mat3::Zero(), true), 0.25, 1e-12);
}

// Backlog G15 (gnss.md section 4.1): with align_yaw_sigma_frontend_cov the yaw variance adds
// sum s_i^2 / sum |a_c,i|^2, s_i^2 the larger horizontal eigenvalue of the front-end antenna
// position covariance J P J^T. Diamond of YawSigmaKnownValue (sum |a_c|^2 = 4, fixes at 1 m^2:
// 0.25 from the fixes):
// - position variance 3 m^2 along w x: 0.25 + 4 x 3 / 4, sigma sqrt(3.25);
// - yaw variance 0.25 rad^2 with the antenna 2 m ahead of b: (2 x 0.5)^2 = 1 m^2 across,
//   0.25 + 4 / 4, sigma sqrt(1.25); the body is yawed by 90 deg so that the rotation of J
//   matters;
// - flag off, or a non-finite P (counted as zero, as the chi-square gate): sigma 0.5.
TEST(GnssAligner, FrontEndCovKnownValue) {
  const Vec3 corners[4] = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(-1, 0, 0), Vec3(0, -1, 0)};
  const auto sigma_with =
      [&](const mss::common::PoseCovariance& P, const Vec3& p_b_g, double body_yaw_rad, bool flag) {
        GnssConfig c = cfg();
        c.align_min_pairs = 4;
        c.align_yaw_sigma_frontend_cov = flag;
        GnssAligner aligner(c);
        mss::common::Result<Pose> res = mss::common::Result<Pose>::error("none");
        for (const Vec3& p : corners) {
          Pose T_w_b = yaw_pose(body_yaw_rad, Vec3::Zero());
          T_w_b.p = p - T_w_b.R * p_b_g;  // antenna at the corner
          res = aligner.update(p, Mat3::Identity(), T_w_b, p_b_g, P);
        }
        EXPECT_TRUE(res.is_ok()) << res.reason();
        return aligner.yaw_sigma_rad();
      };
  mss::common::PoseCovariance P_x = mss::common::PoseCovariance::Zero();
  P_x(0, 0) = 3.0;  // rho_x of the body; body yaw 0, so w x
  EXPECT_NEAR(sigma_with(P_x, Vec3::Zero(), 0.0, true), std::sqrt(3.25), 1e-12);
  EXPECT_NEAR(sigma_with(P_x, Vec3::Zero(), 0.0, false), 0.5, 1e-12);
  mss::common::PoseCovariance P_yaw = mss::common::PoseCovariance::Zero();
  P_yaw(5, 5) = 0.25;
  EXPECT_NEAR(sigma_with(P_yaw, Vec3(2.0, 0.0, 0.0), 0.5 * kPi, true), std::sqrt(1.25), 1e-12);
  mss::common::PoseCovariance P_nan = P_x;
  P_nan(1, 1) = kNan;
  EXPECT_NEAR(sigma_with(P_nan, Vec3::Zero(), 0.0, true), 0.5, 1e-12);
  // A non-finite covariance counts as zero for its fix only: P_nan on the first corner, 3 m^2 on
  // the other three: 0.25 + 3 x 3 / 4, sigma sqrt(2.5).
  {
    GnssConfig c = cfg();
    c.align_min_pairs = 4;
    c.align_yaw_sigma_frontend_cov = true;
    GnssAligner aligner(c);
    mss::common::Result<Pose> res = mss::common::Result<Pose>::error("none");
    for (int i = 0; i < 4; ++i) {
      Pose T_w_b = yaw_pose(0.0, Vec3::Zero());
      T_w_b.p = corners[i];
      res = aligner.update(corners[i], Mat3::Identity(), T_w_b, Vec3::Zero(), i == 0 ? P_nan : P_x);
    }
    EXPECT_TRUE(res.is_ok()) << res.reason();
    EXPECT_NEAR(aligner.yaw_sigma_rad(), std::sqrt(2.5), 1e-12);
  }
  // The 4-argument update equals the 5-argument one with P = 0, flag on or off.
  for (const bool flag : {false, true}) {
    GnssConfig c = cfg();
    c.align_yaw_sigma_frontend_cov = flag;
    GnssAligner a4(c);
    GnssAligner a5(c);
    for (int k = 0; k < 40; ++k) {
      const Pose T_w_b = body_on_line(k, 0.3);
      const Vec3 p = T_w_b * Vec3(-0.2, 0.1, 0.6) + Vec3(0.3 * std::sin(k), 0.2 * std::cos(k), 0);
      const auto r4 = a4.update(p, Mat3::Identity(), T_w_b, Vec3(-0.2, 0.1, 0.6));
      const auto r5 = a5.update(
          p, Mat3::Identity(), T_w_b, Vec3(-0.2, 0.1, 0.6), mss::common::PoseCovariance::Zero());
      ASSERT_EQ(r4.is_ok(), r5.is_ok());
      if (r4.is_ok()) {
        EXPECT_TRUE((r4.value().R.array() == r5.value().R.array()).all());
      }
    }
    EXPECT_EQ(a4.yaw_sigma_rad(), a5.yaw_sigma_rad());
  }
}

// Backlog G15: a front end dead-reckoned along a LiDAR-degenerate axis (corridor seeds 51 and
// 52: the front end ran backwards while the robot drove forwards). Straight drive at 0.5 m/s, the
// w side reversed along the axis, 0.5 m fixes, the yaw check at 6 deg. Taking the front end as
// exact, the aligner aligns about 180 deg off with a sigma of a few degrees; with the front-end
// covariance of the corridor's scan matching (1000 m^2 along the unobserved axis) the sigma
// stays far above 6 deg (31.6 m over the spread) and it never aligns. Counting the front-end
// variance as independent per fix instead would average it down: aligned at fix 478, 180 deg off
// with a 6 deg sigma.
TEST(GnssAligner, FrontEndCovCatchesADeadReckonedAxis) {
  const Pose T_e_w = yaw_pose(0.4, Vec3(3.0, -2.0, 0.0));
  for (const bool flag : {false, true}) {
    GnssConfig c = cfg();
    c.align_check_yaw_sigma = true;
    c.align_yaw_max_sigma_rad = 6.0 * kDeg;
    c.align_yaw_sigma_frontend_cov = flag;
    GnssAligner aligner(c);
    std::mt19937 rng(7);
    std::normal_distribution<double> n01(0.0, 1.0);
    mss::common::PoseCovariance P = mss::common::PoseCovariance::Zero();
    P(0, 0) = 1000.0;  // unobserved_var_m2 of the corridor runs, along the axis (body x = w x)
    P(1, 1) = 1e-4;
    P(2, 2) = 1e-4;
    int first = -1;
    double err = 0.0;
    double sigma = 0.0;
    for (int k = 0; k < 600 && first < 0; ++k) {
      const Pose truth = yaw_pose(0.0, Vec3(0.1 * k, 0.0, 0.0));
      const Pose front = yaw_pose(0.0, Vec3(-0.1 * k, 0.0, 0.0));  // dead reckoning backwards
      Vec3 p = T_e_w * (truth * Vec3::Zero());
      p.x() += 0.5 * n01(rng);
      p.y() += 0.5 * n01(rng);
      const auto r = aligner.update(p, Mat3::Identity() * 0.25, front, Vec3::Zero(), P);
      if (r.is_ok()) {
        first = k;
        err = wrap_pi(yaw_of(r.value().R) - 0.4);
        sigma = aligner.yaw_sigma_rad();
      }
    }
    std::printf(
        "[ G15 ] dead-reckoned axis, front-end covariance %d: first fit at fix %d, %.1f "
        "deg off, sigma %.2f deg; last sigma %.1f deg\n",
        flag ? 1 : 0,
        first,
        err / kDeg,
        sigma / kDeg,
        aligner.yaw_sigma_rad() / kDeg);
    if (flag) {
      EXPECT_EQ(first, -1);
      EXPECT_GT(aligner.yaw_sigma_rad(), 30.0 * kDeg);
    } else {
      ASSERT_GE(first, 0);  // negative control: confident and 180 deg off
      EXPECT_GT(std::abs(err), 170.0 * kDeg);
      EXPECT_LT(sigma, 6.0 * kDeg);
    }
  }
}

TEST(GnssAligner, NonFiniteCovarianceNotAdded) {
  GnssConfig c = cfg();
  c.align_min_pairs = 1;
  GnssAligner aligner(c);
  Mat3 bad = Mat3::Identity();
  bad(1, 0) = kNan;
  EXPECT_EQ(aligner.update(Vec3::Zero(), bad, yaw_pose(0.0, Vec3::Zero()), Vec3::Zero()).reason(),
            "non-finite input");
  // Not added: the next pair is the first one (no spread), and the sums stay finite.
  EXPECT_EQ(
      aligner.update(Vec3::Zero(), Mat3::Identity(), yaw_pose(0.0, Vec3::Zero()), Vec3::Zero())
          .reason(),
      "insufficient horizontal spread");
}

// Portable normal deviates (splitmix64 + Box-Muller): the same values with libstdc++ and
// libc++, so a seed reproduces a known case on both.
class PortableNormal {
 public:
  explicit PortableNormal(std::uint64_t seed) : state_(seed) {}
  double operator()() {
    if (have_spare_) {
      have_spare_ = false;
      return spare_;
    }
    const double u1 = (static_cast<double>(next() >> 11U) + 0.5) * 0x1.0p-53;
    const double u2 = static_cast<double>(next() >> 11U) * 0x1.0p-53;
    const double r = std::sqrt(-2.0 * std::log(u1));
    spare_ = r * std::sin(2.0 * kPi * u2);
    have_spare_ = true;
    return r * std::cos(2.0 * kPi * u2);
  }

 private:
  std::uint64_t next() {
    std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31U);
  }
  std::uint64_t state_;
  double spare_ = 0.0;
  bool have_spare_ = false;
};

// E-01-like run as the aligner sees it (gnss_fusion_study.md section 17): from keyframe 0 at
// 2 s, 3 s at rest, a 2 s ramp to 0.5 m/s, then a circle of R = 0.5 / 0.3 = 1.67 m with the
// body yaw along the path, the lever arm of config/sim/warehouse.yaml, 5 Hz fixes with white
// 1.0 m horizontal noise declared as such (the bag covariance). The front end is exact.
constexpr double kE01DtS = 0.2;          // s, 5 Hz
constexpr double kE01RestS = 3.0;        // s, rest after keyframe 0
constexpr double kE01RampS = 2.0;        // s
constexpr double kE01SpeedMps = 0.5;     // m/s
constexpr double kE01OmegaRadps = 0.3;   // rad/s
constexpr double kE01SigmaM = 1.0;       // m, horizontal fix sigma
constexpr double kE01SigmaVM = 2.0;      // m, vertical fix sigma (declared only)
constexpr double kE01MaxSigmaDeg = 6.0;  // config/default.yaml gnss.align_yaw_max_sigma_deg
constexpr double kE01YawTrue = 30.0 * kDeg;
constexpr int kE01MaxFixes = 300;  // 60 s

Pose e01_body(int k) {
  const double t = kE01DtS * k;
  const double tm = std::max(0.0, t - kE01RestS);  // time since the motion started
  // Arc length: v = V tm / T_r during the ramp, then V; heading = s / R.
  const double s = tm < kE01RampS
                       ? 0.5 * kE01SpeedMps * tm * tm / kE01RampS
                       : 0.5 * kE01SpeedMps * kE01RampS + kE01SpeedMps * (tm - kE01RampS);
  const double radius = kE01SpeedMps / kE01OmegaRadps;
  const double th = s / radius;
  return yaw_pose(th, Vec3(radius * std::sin(th), radius * (1.0 - std::cos(th)), 0.0));
}

struct FirstAlignment {
  int n = 0;  // fixes fed when the aligner first returned T_e_w (0: never)
  double err_rad = 0.0;
  double sigma_rad = 0.0;
};

FirstAlignment e01_first_alignment(std::uint64_t seed, bool check) {
  GnssConfig c = cfg();
  c.align_check_yaw_sigma = check;
  c.align_yaw_max_sigma_rad = kE01MaxSigmaDeg * kDeg;
  c.align_yaw_min_var_m2 = 0.25;
  GnssAligner aligner(c);
  PortableNormal noise(seed);
  const Pose T_e_w = yaw_pose(kE01YawTrue, Vec3(120.0, -45.0, 3.0));
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  Mat3 cov = Mat3::Identity() * (kE01SigmaM * kE01SigmaM);
  cov(2, 2) = kE01SigmaVM * kE01SigmaVM;
  for (int k = 0; k < kE01MaxFixes; ++k) {
    const Pose T_w_b = e01_body(k);
    const double nx = kE01SigmaM * noise();
    const double ny = kE01SigmaM * noise();
    const Vec3 p_e_g = T_e_w * (T_w_b * p_b_g) + Vec3(nx, ny, 0.0);
    const auto res = aligner.update(p_e_g, cov, T_w_b, p_b_g);
    if (res.is_ok()) {
      return {k + 1, wrap_pi(yaw_of(res.value().R) - kE01YawTrue), aligner.yaw_sigma_rad()};
    }
  }
  return {};
}

// G1 reproduction: with the default (no yaw check) the aligner accepts the first fit whose
// spread passes 1 m, 11 s after keyframe 0 as on E-01 (13 s since the first record), with
// a 7.6 deg sigma and, for this seed, the yaw about 21 deg off as on E-01. With the check it
// waits until the sigma is at most 6 deg, and the yaw is then within 3 sigma.
TEST(GnssAligner, E01LikeUnsupportedYawRefused) {
  constexpr std::uint64_t kSeed = 129;  // 20.7 deg off without the check
  const FirstAlignment off = e01_first_alignment(kSeed, false);
  ASSERT_GT(off.n, 0);
  EXPECT_EQ(off.n, 56);
  EXPECT_NEAR(off.sigma_rad, 7.6 * kDeg, 0.2 * kDeg);
  EXPECT_GT(std::abs(off.err_rad), 18.0 * kDeg);  // beyond 3 x the 6 deg threshold
  const FirstAlignment on = e01_first_alignment(kSeed, true);
  ASSERT_GT(on.n, off.n);
  EXPECT_LE(on.sigma_rad, kE01MaxSigmaDeg * kDeg);
  EXPECT_LT(std::abs(on.err_rad), 3.0 * on.sigma_rad);
}

// The predicted sigma is honest on this geometry (white, declared noise): over 400 seeds the
// yaw error at the first alignment, normalised by its sigma, has a mean square near 1, with
// and without the check. Without it more than 1 run in 100 starts beyond 18 deg (3 x 6 deg),
// with it none.
TEST(GnssAligner, E01LikeSigmaConsistent) {
  constexpr int kRuns = 400;
  for (const bool check : {false, true}) {
    double nees = 0.0;
    int beyond_18 = 0;
    int beyond_3s = 0;
    for (int i = 0; i < kRuns; ++i) {
      const FirstAlignment a = e01_first_alignment(1000U + static_cast<std::uint64_t>(i), check);
      ASSERT_GT(a.n, 0);
      const double z = a.err_rad / a.sigma_rad;
      nees += z * z;
      beyond_18 += std::abs(a.err_rad) > 18.0 * kDeg ? 1 : 0;
      beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
      if (check) {
        EXPECT_LE(a.sigma_rad, kE01MaxSigmaDeg * kDeg);
      }
    }
    nees /= kRuns;
    std::printf("check %d: yaw NEES %.3f, beyond 18 deg %d, beyond 3 sigma %d of %d\n",
                check ? 1 : 0,
                nees,
                beyond_18,
                beyond_3s,
                kRuns);
    EXPECT_GT(nees, 0.8) << "check " << check;
    EXPECT_LT(nees, 1.25) << "check " << check;
    if (check) {
      EXPECT_EQ(beyond_18, 0);
    } else {
      EXPECT_GT(beyond_18, kRuns / 100);
    }
  }
}

TEST(GnssAligner, YawSigmaCheckErrorOrder) {
  // The check runs after the spread check and the pair is added, so the fit aligns once the
  // spread is large enough.
  GnssConfig c = cfg();
  c.align_check_yaw_sigma = true;
  c.align_yaw_max_sigma_rad = 1.0 * kDeg;
  c.align_yaw_min_var_m2 = 0.25;
  GnssAligner aligner(c);
  int n_ok_at = 0;
  for (int n = 1; n <= 400 && n_ok_at == 0; ++n) {
    const Pose T_w_b = body_on_line(n - 1, 0.0);
    const auto res = aligner.update(T_w_b.p, Mat3::Identity(), T_w_b, Vec3::Zero());
    if (n <= 9) {
      EXPECT_EQ(res.reason(), "not enough pairs") << "n " << n;
    } else if (n <= 17) {
      EXPECT_EQ(res.reason(), "insufficient horizontal spread") << "n " << n;
    } else if (!res.is_ok()) {
      EXPECT_EQ(res.reason(), "yaw sigma above threshold") << "n " << n;
      EXPECT_GT(aligner.yaw_sigma_rad(), 1.0 * kDeg);
    } else {
      n_ok_at = n;
      EXPECT_LE(aligner.yaw_sigma_rad(), 1.0 * kDeg);
    }
  }
  // 1 m^2 fixes on the 0.2 m-step line: sum |a_c|^2 = 0.04 (n^3 - n) / 12 m^2 and
  // sigma = 1 / sqrt(that), <= 1 deg (3283 m^2) from n = 100 on.
  EXPECT_EQ(n_ok_at, 100);
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
    n_ok += aligner.update(T_e_w.p, Mat3::Identity(), T_e_w, p_b_g).is_ok() ? 1 : 0;
    n_ok +=
        aligner.update(
                   T_e_w.p, Mat3::Identity(), T_e_w, p_b_g, mss::common::PoseCovariance::Identity())
                .is_ok()
            ? 1
            : 0;
  }
  EXPECT_EQ(AllocCounter::count(), 0u);
  EXPECT_EQ(n_ok, 102);
}

}  // namespace
