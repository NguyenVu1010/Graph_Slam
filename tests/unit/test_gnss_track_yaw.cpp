// Unit tests of gnss::GnssTrackYawAligner (ADR-023, docs/derivations/gnss.md section 5): the yaw
// of T_e_w from the GNSS track direction against the body heading, known answers on synthetic
// tracks with fixed seeds.
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
#include "mss/gnss/config.hpp"
#include "mss/gnss/gnss.hpp"

#include "alloc_counter.hpp"

namespace {

using mss::common::Mat3;
using mss::common::Pose;
using mss::common::Vec3;
using mss::gnss::GnssAligner;
using mss::gnss::GnssConfig;
using mss::gnss::GnssTrackYawAligner;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kRateHz = 5.0;      // fixes per second (ADR-011)
constexpr double kSpeedMps = 0.5;    // m/s, as the corridor scene
constexpr double kSigmaM = 0.5;      // m, fix noise per axis (brief)
const Vec3 kPbg(-0.20, 0.10, 0.60);  // m, lever arm in b (box world)

double wrap_pi(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

double yaw_of(const Mat3& R) {
  return std::atan2(R(1, 0), R(0, 0));
}

Pose yaw_pose(double yaw_rad, const Vec3& p) {
  Pose T;
  T.R = Eigen::AngleAxisd(yaw_rad, Vec3::UnitZ()).toRotationMatrix();
  T.p = p;
  return T;
}

const Pose kTew = yaw_pose(30.0 * kDeg, Vec3(12.0, -7.0, 1.0));  // true T_e_w

GnssConfig track_cfg() {
  GnssConfig c;
  c.enabled = true;
  c.align_yaw_from_track = true;  // defaults: 10 fixes per chunk, 3 deg, 3 sigmas
  // Short tracks for the geometry tests; the 5 m default is tested in MinTrackLength.
  c.align_yaw_min_track_m = 0.5;
  // No variance floor for the geometry tests (declared 1 mm fixes); the 0.25 m^2 default is
  // tested in VarianceFloor.
  c.align_yaw_min_var_m2 = 1e-9;
  return c;
}

Mat3 fix_cov(double sigma_m) {
  return Mat3::Identity() * sigma_m * sigma_m;
}

// One synthetic run: body pose at fix k (k / kRateHz seconds).
struct Track {
  virtual ~Track() = default;
  [[nodiscard]] virtual Pose body(int k) const = 0;
};

// Straight line in w with heading `heading_rad`, `static_fixes` at rest first, then forward at
// kSpeedMps; `reverse_from` >= 0: from that fix on, back along the same line (same heading).
struct Line : Track {
  double heading_rad = 0.0;
  int static_fixes = 0;
  int reverse_from = -1;
  [[nodiscard]] Pose body(int k) const override {
    const double step = kSpeedMps / kRateHz;
    const int moving = std::max(0, k - static_fixes);
    double s = step * moving;
    if (reverse_from >= 0 && k > reverse_from) {
      const double s_turn = step * std::max(0, reverse_from - static_fixes);
      s = s_turn - step * (k - reverse_from);
    }
    const Vec3 dir(std::cos(heading_rad), std::sin(heading_rad), 0.0);
    return yaw_pose(heading_rad, Vec3(3.0, -2.0, 0.0) + s * dir);
  }
};

// Straight for `straight_fixes`, then a left arc of radius `radius_m` (heading tangent).
struct Turn : Track {
  int straight_fixes = 50;
  double radius_m = 5.0;
  [[nodiscard]] Pose body(int k) const override {
    const double step = kSpeedMps / kRateHz;
    if (k <= straight_fixes) {
      return yaw_pose(0.0, Vec3(step * k, 0.0, 0.0));
    }
    const double x0 = step * straight_fixes;
    const double psi = step * (k - straight_fixes) / radius_m;
    return yaw_pose(psi,
                    Vec3(x0 + radius_m * std::sin(psi), radius_m * (1.0 - std::cos(psi)), 0.0));
  }
};

// Fix k of `track` in e, with N(0, sigma^2) noise per axis from `rng` (nullptr: noise-free).
Vec3 fix_at(const Track& track, int k, double sigma_m, std::mt19937* rng) {
  Vec3 p = kTew * (track.body(k) * kPbg);
  if (rng != nullptr) {
    std::normal_distribution<double> n01(0.0, 1.0);
    for (int i = 0; i < 3; ++i) {
      p(i) += sigma_m * n01(*rng);
    }
  }
  return p;
}

// Feeds fixes 0..n-1; returns the last result.
mss::common::Result<Pose> feed(
    GnssTrackYawAligner& a, const Track& track, int n, double sigma_m, std::mt19937* rng) {
  mss::common::Result<Pose> r = mss::common::Result<Pose>::error("no fix");
  for (int k = 0; k < n; ++k) {
    r = a.update(fix_at(track, k, sigma_m, rng), fix_cov(sigma_m), track.body(k), kPbg);
  }
  return r;
}

// Signed forward wheel travel from fix k - 1 to fix k (m): the body displacement along x_b.
double wheel_travel(const Track& track, int k) {
  if (k == 0) {
    return 0.0;
  }
  const Pose b = track.body(k);
  return (b.p - track.body(k - 1).p).dot(b.R.col(0));
}

TEST(GnssTrackYaw, NoiseFreeStraightLineIsExact) {
  for (const double heading_deg : {0.0, 20.0, -135.0, 179.0}) {
    GnssTrackYawAligner a(track_cfg());
    Line line;
    line.heading_rad = heading_deg * kDeg;
    const auto r = feed(a, line, 60, 1e-3, nullptr);  // 1 mm declared sigma: aligns early
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_NEAR(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R)), 0.0, 1e-9) << heading_deg;
    EXPECT_LT((r.value().p - kTew.p).norm(), 1e-9) << heading_deg;
    EXPECT_EQ(a.n_reversals(), 0);
  }
}

// The 180 deg case of the brief: true yaw of T_e_w 180 deg is recovered as 180 deg, not 0: the
// sign comes from the GNSS displacement along the heading (forward motion).
TEST(GnssTrackYaw, YawNear180FromDisplacementSign) {
  for (const double yaw_deg : {180.0, -179.0, 179.0}) {
    const Pose T_e_w = yaw_pose(yaw_deg * kDeg, Vec3(1.0, 2.0, 0.0));
    GnssTrackYawAligner a(track_cfg());
    Line line;
    mss::common::Result<Pose> r = mss::common::Result<Pose>::error("none");
    for (int k = 0; k < 60; ++k) {
      r = a.update(T_e_w * (line.body(k) * kPbg), fix_cov(1e-3), line.body(k), kPbg);
    }
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_NEAR(wrap_pi(yaw_of(r.value().R) - yaw_deg * kDeg), 0.0, 1e-9) << yaw_deg;
  }
}

// The corridor failure (gnss_fusion_study.md section 13, seeds 51 / 52): the front end's
// heading is right but its along-axis positions run backwards (dead reckoning with a velocity
// of the wrong sign). GnssAligner fits those positions and flips by 180 deg; the track aligner
// does not use them for the yaw.
TEST(GnssTrackYaw, BackwardDeadReckoningDoesNotFlip) {
  GnssConfig cfg = track_cfg();
  GnssTrackYawAligner track(cfg);
  GnssAligner positions(cfg);
  std::mt19937 rng(51U);
  Line truth;
  truth.static_fixes = 10;
  mss::common::Result<Pose> rt = mss::common::Result<Pose>::error("none");
  mss::common::Result<Pose> rp = mss::common::Result<Pose>::error("none");
  for (int k = 0; k < 150; ++k) {
    const Vec3 p_e = fix_at(truth, k, kSigmaM, &rng);
    Pose fe = truth.body(k);
    fe.p.x() = 3.0 - 0.6 * (fe.p.x() - 3.0);  // front end: along-axis motion reversed, scaled
    rt = track.update(p_e, fix_cov(kSigmaM), fe, kPbg);
    rp = positions.update(p_e, fe, kPbg);
  }
  ASSERT_TRUE(rt.is_ok()) << rt.reason();
  ASSERT_TRUE(rp.is_ok()) << rp.reason();
  const double err_track = wrap_pi(yaw_of(rt.value().R) - yaw_of(kTew.R));
  const double err_pos = wrap_pi(yaw_of(rp.value().R) - yaw_of(kTew.R));
  EXPECT_LT(std::abs(err_track), 3.0 * track.yaw_sigma_rad());
  EXPECT_GT(std::abs(err_pos), 170.0 * kDeg);  // negative control: the position fit flips
}

// 0.5 m noise, fixed seeds: the predicted sigma of the first transform is consistent: the mean
// normalised squared error is near 1 (chi-square mean of 1 DoF; 200 runs give a standard error
// of about 0.1), and few runs are beyond 3 sigma (0.27 % expected, so 0.5 of 200; at most 3
// allowed, a binomial tail of about 1 %; measured 0) and none beyond 4 sigma.
TEST(GnssTrackYaw, NoisyStraightLineWithinPredictedSigma) {
  constexpr int kRuns = 200;
  double nees_sum = 0.0;
  int beyond_3s = 0;
  for (int run = 0; run < kRuns; ++run) {
    std::mt19937 rng(1000U + static_cast<std::uint32_t>(run));
    GnssTrackYawAligner a(track_cfg());
    Line line;
    line.heading_rad = 0.3 * run;
    line.static_fixes = 10;
    bool aligned = false;
    for (int k = 0; k < 400 && !aligned; ++k) {
      const auto r = a.update(fix_at(line, k, kSigmaM, &rng), fix_cov(kSigmaM), line.body(k), kPbg);
      if (r.is_ok()) {
        aligned = true;
        const double err = wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R));
        const double sigma = a.yaw_sigma_rad();
        EXPECT_LE(sigma, track_cfg().align_yaw_max_sigma_rad);
        EXPECT_LT(std::abs(err), 4.0 * sigma) << "run " << run;
        beyond_3s += std::abs(err) > 3.0 * sigma ? 1 : 0;
        nees_sum += err * err / (sigma * sigma);
      }
    }
    ASSERT_TRUE(aligned) << "run " << run;
  }
  const double nees_mean = nees_sum / kRuns;
  std::printf("[ track yaw ] mean normalised squared yaw error %.3f, %d of %d beyond 3 sigma\n",
              nees_mean,
              beyond_3s,
              kRuns);
  EXPECT_LE(beyond_3s, 3);
  EXPECT_GT(nees_mean, 0.7);
  EXPECT_LT(nees_mean, 1.3);
}

// A 90 deg turn inside the window: each segment uses its own heading, so the yaw stays right
// (noise-free; the residual is the chord-vs-mean-heading and lever-arm approximation).
TEST(GnssTrackYaw, TurnInWindow) {
  GnssTrackYawAligner a(track_cfg());
  Turn turn;
  turn.straight_fixes = 0;  // the whole window is the arc: 5 m radius, 0.1 m per fix
  const int quarter = static_cast<int>(0.5 * kPi * turn.radius_m / (kSpeedMps / kRateHz));
  const auto r = feed(a, turn, quarter, 1e-3, nullptr);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 0.5 * kDeg);
  EXPECT_EQ(a.n_reversals(), 0);
}

// With 0.5 m noise and a turn after 10 m straight, the yaw is within 3 sigma.
TEST(GnssTrackYaw, NoisyTurnWithinSigma) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    GnssTrackYawAligner a(track_cfg());
    Turn turn;
    turn.straight_fixes = 100;
    const auto r = feed(a, turn, 300, kSigmaM, &rng);
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 3.0 * a.yaw_sigma_rad())
        << "seed " << seed;
  }
}

// Forward 20 m then back 20 m along the same line with the same heading: once the yaw is
// confident the backward segments are detected and negated, so the yaw stays right. Without
// that (a reversal from the very start) the forward-motion assumption gives 180 deg: the
// documented limitation.
TEST(GnssTrackYaw, ReversalAfterAlignmentResolvedBySign) {
  std::mt19937 rng(7U);
  GnssTrackYawAligner a(track_cfg());
  Line line;
  line.reverse_from = 200;  // 20 m forward, then backwards
  const auto r = feed(a, line, 400, kSigmaM, &rng);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_GT(a.n_reversals(), 5);
  EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 3.0 * a.yaw_sigma_rad());

  GnssTrackYawAligner b(track_cfg());
  Line back;
  back.reverse_from = 0;  // backwards from the start
  const auto rb = feed(b, back, 200, 1e-3, nullptr);
  ASSERT_TRUE(rb.is_ok()) << rb.reason();
  EXPECT_NEAR(std::abs(wrap_pi(yaw_of(rb.value().R) - yaw_of(kTew.R))), kPi, 1e-9);
}

TEST(GnssTrackYaw, TooLittleMotionNoTransform) {
  // At rest: the chunk means differ by noise only.
  std::mt19937 rng(3U);
  GnssTrackYawAligner a(track_cfg());
  Line still;
  still.static_fixes = 100000;
  for (int k = 0; k < 600; ++k) {
    const auto r = a.update(fix_at(still, k, kSigmaM, &rng), fix_cov(kSigmaM), still.body(k), kPbg);
    ASSERT_FALSE(r.is_ok()) << "k " << k;
    EXPECT_TRUE(r.reason() == "not enough track" || r.reason() == "track too short" ||
                r.reason() == "track not coherent" || r.reason() == "yaw sigma too large")
        << r.reason();
  }
  EXPECT_GT(a.yaw_sigma_rad(), track_cfg().align_yaw_max_sigma_rad);
  // A frozen fix while the body moves: no along-track sum, never aligned.
  GnssTrackYawAligner b(track_cfg());
  Line line;
  for (int k = 0; k < 300; ++k) {
    EXPECT_FALSE(b.update(Vec3(5.0, 5.0, 0.0), fix_cov(kSigmaM), line.body(k), kPbg).is_ok());
  }
}

TEST(GnssTrackYaw, ErrorSequence) {
  GnssTrackYawAligner a(track_cfg());
  Line line;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(a.update(Vec3(nan, 0.0, 0.0), fix_cov(1e-3), line.body(0), kPbg).reason(),
            "non-finite input");
  Mat3 bad_cov = fix_cov(1e-3);
  bad_cov(0, 0) = nan;
  EXPECT_EQ(a.update(Vec3::Zero(), bad_cov, line.body(0), kPbg).reason(), "non-finite input");
  EXPECT_EQ(a.update(Vec3::Zero(), Mat3::Zero(), line.body(0), kPbg).reason(),
            "non-positive covariance");
  Pose up = line.body(0);
  up.R = Eigen::AngleAxisd(-0.5 * kPi, Vec3::UnitY()).toRotationMatrix();  // x_b straight up
  EXPECT_EQ(a.update(Vec3::Zero(), fix_cov(1e-3), up, kPbg).reason(), "heading undefined");
  // 10 fixes per chunk: no segment before fix 20 (index 19).
  for (int k = 0; k < 19; ++k) {
    EXPECT_EQ(a.update(fix_at(line, k, 0.0, nullptr), fix_cov(1e-3), line.body(k), kPbg).reason(),
              "not enough track")
        << k;
  }
  EXPECT_TRUE(a.update(fix_at(line, 19, 0.0, nullptr), fix_cov(1e-3), line.body(19), kPbg).is_ok());
  // A large declared sigma: the same track is not enough.
  GnssTrackYawAligner b(track_cfg());
  for (int k = 0; k < 20; ++k) {
    const auto r = b.update(fix_at(line, k, 0.0, nullptr), fix_cov(kSigmaM), line.body(k), kPbg);
    EXPECT_FALSE(r.is_ok());
    if (k == 19) {
      EXPECT_EQ(r.reason(), "yaw sigma too large");
    }
  }
}

TEST(GnssTrackYaw, UsesConfiguredThresholds) {
  GnssConfig loose = track_cfg();
  loose.align_yaw_max_sigma_rad = 10.0 * kDeg;
  GnssConfig tight = track_cfg();
  std::mt19937 rng_a(11U);
  std::mt19937 rng_b(11U);
  GnssTrackYawAligner a(loose);
  GnssTrackYawAligner b(tight);
  Line line;
  int first_a = -1;
  int first_b = -1;
  for (int k = 0; k < 400; ++k) {
    if (a.update(fix_at(line, k, kSigmaM, &rng_a), fix_cov(kSigmaM), line.body(k), kPbg).is_ok() &&
        first_a < 0) {
      first_a = k;
    }
    if (b.update(fix_at(line, k, kSigmaM, &rng_b), fix_cov(kSigmaM), line.body(k), kPbg).is_ok() &&
        first_b < 0) {
      first_b = k;
    }
  }
  ASSERT_GE(first_a, 0);
  ASSERT_GE(first_b, 0);
  EXPECT_LT(first_a, first_b);
  // Chunk size: with 4 fixes per chunk the first segment exists after 8 fixes.
  GnssConfig small = track_cfg();
  small.align_yaw_chunk_fixes = 4;
  small.align_yaw_min_track_m = 0.1;  // the first segment is 0.4 m
  GnssTrackYawAligner c(small);
  for (int k = 0; k < 8; ++k) {
    const auto r = c.update(fix_at(line, k, 0.0, nullptr), fix_cov(1e-3), line.body(k), kPbg);
    EXPECT_EQ(r.is_ok(), k == 7) << k << " " << r.reason();
  }
}

// Review I1 (a): fast turns. With 10 fixes per chunk a yaw rate above pi / 10 rad per fix turned
// the bisector of the two chunk headings backwards (exact 180 deg flip at a sigma below 0.5 deg).
// The segment direction now comes from the unwrapped heading, and segments turning by pi or more
// are skipped: noise-free circles at 0.25 to 0.6 rad per fix either never align or are exact.
TEST(GnssTrackYaw, FastTurnNeverFlips) {
  for (const double radius_m : {1.0, 0.3}) {
    for (const double rate : {0.03, 0.07, 0.25, 0.33, 0.60}) {  // rad per fix
      for (const Vec3& lever : {Vec3(0.0, 0.0, 0.0), kPbg}) {
        GnssTrackYawAligner a(track_cfg());
        int n_ok = 0;
        double worst_rad = 0.0;
        for (int k = 0; k < 600; ++k) {
          const double psi = rate * k;
          const Pose b =
              yaw_pose(psi, Vec3(radius_m * std::sin(psi), radius_m * (1.0 - std::cos(psi)), 0.0));
          const auto r = a.update(kTew * (b * lever), fix_cov(0.1), b, lever);
          if (r.is_ok()) {
            ++n_ok;
            worst_rad =
                std::max(worst_rad, std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))));
          }
        }
        // Segment turn 10 x rate against the 45 deg gate (0.785 rad): 0.03 and 0.07 pass.
        if (10.0 * rate > track_cfg().align_yaw_max_turn_rad) {
          EXPECT_EQ(n_ok, 0) << radius_m << " " << rate;  // every segment gated
          EXPECT_GT(a.n_skipped_turns(), 50);
        } else {
          EXPECT_GT(n_ok, 0) << radius_m << " " << rate;
          EXPECT_LT(worst_rad, 1e-6) << radius_m << " " << rate;
        }
      }
    }
  }
}

// Review I1 (b): tight arcs with the box-world lever arm. The fixed-point lever loop left
// -3.96 deg at R = 0.2 m (sigma about 1 deg); the closed form is exact on a constant arc.
TEST(GnssTrackYaw, TightArcWithLeverArmExact) {
  for (const double radius_m : {0.2, 0.3, 2.0}) {
    GnssTrackYawAligner a(track_cfg());
    mss::common::Result<Pose> r = mss::common::Result<Pose>::error("none");
    for (int k = 0; k < 600; ++k) {
      const double psi = 0.05 * k;  // rad, 0.5 rad (29 deg) per chunk, inside the turn gate
      const Pose b =
          yaw_pose(psi, Vec3(radius_m * std::sin(psi), radius_m * (1.0 - std::cos(psi)), 0.0));
      r = a.update(kTew * (b * kPbg), fix_cov(0.01), b, kPbg);
    }
    ASSERT_TRUE(r.is_ok()) << radius_m << " " << r.reason();
    EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 1e-9) << radius_m;
  }
  // Noisy (0.5 m) on a 2 m circle at 0.5 m/s with the lever arm: within 3 sigma, 20 seeds.
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    GnssTrackYawAligner a(track_cfg());
    mss::common::Result<Pose> r = mss::common::Result<Pose>::error("none");
    for (int k = 0; k < 600; ++k) {
      const double psi = 0.05 * k;  // 0.1 m per fix on a 2 m circle
      const Pose b = yaw_pose(psi, Vec3(2.0 * std::sin(psi), 2.0 * (1.0 - std::cos(psi)), 0.0));
      Vec3 p = kTew * (b * kPbg);
      for (int i = 0; i < 3; ++i) {
        p(i) += kSigmaM * n01(rng);
      }
      r = a.update(p, fix_cov(kSigmaM), b, kPbg);
    }
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 3.0 * a.yaw_sigma_rad())
        << "seed " << seed;
  }
}

// Review I2: the forward-motion assumption must hold over the window before the estimate is
// confident. A short move forward (3 m) followed by a longer one backwards makes the net
// along-heading displacement backwards: the aligner converges to yaw + 180 deg and then reads
// later forward motion as reversals. This pins that documented behaviour (ADR-023): with GNSS
// and heading alone it cannot be told from forward motion at yaw + 180 deg.
TEST(GnssTrackYaw, NetBackwardMotionBeforeConfidenceFlips) {
  std::mt19937 rng(1U);
  GnssConfig cfg = track_cfg();
  cfg.align_yaw_min_track_m = 5.0;  // the default
  GnssTrackYawAligner a(cfg);
  Line line;
  line.reverse_from = 30;  // 3 m forward, then backwards
  int first = -1;
  double err_first = 0.0;
  for (int k = 0; k < 300 && first < 0; ++k) {
    const auto r = a.update(fix_at(line, k, kSigmaM, &rng), fix_cov(kSigmaM), line.body(k), kPbg);
    if (r.is_ok()) {
      first = k;
      err_first = wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R));
    }
  }
  ASSERT_GE(first, 0);
  EXPECT_GT(std::abs(err_first), kPi - 3.0 * a.yaw_sigma_rad());
  EXPECT_GT(a.n_no_wheel_segments(), 0);
}

// Addendum ruling 1: with wheel data the direction of travel of each segment comes from the
// sign of the wheel travel, so the same forward-then-back run aligns correctly and the
// fallback is never used. Also from the start backwards.
TEST(GnssTrackYaw, WheelSignResolvesBackwardMotion) {
  for (const int reverse_from : {30, 0}) {
    std::mt19937 rng(1U);
    GnssConfig cfg = track_cfg();
    cfg.align_yaw_min_track_m = 5.0;
    GnssTrackYawAligner a(cfg);
    Line line;
    line.reverse_from = reverse_from;
    mss::common::Result<Pose> r = mss::common::Result<Pose>::error("none");
    int first = -1;
    double err_first = 0.0;
    for (int k = 0; k < 300; ++k) {
      r = a.update(fix_at(line, k, kSigmaM, &rng),
                   fix_cov(kSigmaM),
                   line.body(k),
                   kPbg,
                   wheel_travel(line, k));
      if (r.is_ok() && first < 0) {
        first = k;
        err_first = wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R));
        EXPECT_LT(std::abs(err_first), 3.0 * a.yaw_sigma_rad()) << reverse_from;
      }
    }
    ASSERT_GE(first, 0) << reverse_from;
    ASSERT_TRUE(r.is_ok());
    EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 3.0 * a.yaw_sigma_rad());
    EXPECT_EQ(a.n_no_wheel_segments(), 0);
    EXPECT_GT(a.n_reversals(), 0);
  }
  // A wheel-data gap (NaN) inside the window falls back for the segments it touches only.
  GnssTrackYawAligner b(track_cfg());
  Line line;
  for (int k = 0; k < 100; ++k) {
    const double w =
        (k >= 40 && k < 45) ? std::numeric_limits<double>::quiet_NaN() : wheel_travel(line, k);
    (void)b.update(fix_at(line, k, 0.0, nullptr), fix_cov(1e-3), line.body(k), kPbg, w);
  }
  EXPECT_EQ(b.n_no_wheel_segments(), 1);
}

// E-01 finding of the fix round: wheel noise at rest has a random sign. Taken as a direction it
// alternates a_k over the static segments, adds 4 var per sign change to the yaw noise and
// delayed E-01's alignment to the end of the run (8 GNSS factors instead of 65). Segments below
// align_yaw_min_wheel_travel_m are skipped as stationary: a static start with noisy wheel
// speed aligns at the same fix as with a clean wheel (and as without wheel data).
TEST(GnssTrackYaw, StationaryWheelNoiseDoesNotDelay) {
  int first[3] = {-1, -1, -1};  // clean wheel, noisy wheel, no wheel
  for (int mode = 0; mode < 3; ++mode) {
    std::mt19937 rng(4U);
    std::mt19937 wheel_rng(5U);
    std::normal_distribution<double> n01(0.0, 1.0);
    GnssTrackYawAligner a(track_cfg());
    Line line;
    line.static_fixes = 30;  // 6 s at rest
    for (int k = 0; k < 400 && first[mode] < 0; ++k) {
      double w = wheel_travel(line, k);
      if (mode == 1) {
        w += 0.005 * n01(wheel_rng);  // 5 mm of wheel noise per fix interval
      } else if (mode == 2) {
        w = std::numeric_limits<double>::quiet_NaN();
      }
      if (a.update(fix_at(line, k, kSigmaM, &rng), fix_cov(kSigmaM), line.body(k), kPbg, w)
              .is_ok()) {
        first[mode] = k;
      }
    }
    if (mode == 1) {
      EXPECT_GT(a.n_stationary_segments(), 0);
    }
  }
  ASSERT_GE(first[0], 0);
  EXPECT_EQ(first[1], first[0]);
  EXPECT_LE(std::abs(first[2] - first[0]), 10);  // within one chunk of the no-wheel case
}

// Addendum ruling 3: the declared variance is floored (default 0.25 m^2). A receiver that
// declares 1 cm while its error is 0.5 m: without the floor the yaw sigma is 50 times too
// small and the aligner commits on the second chunk; with it the first transform is within
// 3 sigma on most runs. 100 seeded runs, counts printed and bounded.
TEST(GnssTrackYaw, VarianceFloor) {
  for (const bool floor_on : {false, true}) {
    int beyond_10deg = 0;
    int beyond_3s = 0;
    for (std::uint32_t seed = 1; seed <= 100; ++seed) {
      std::mt19937 rng(seed);
      GnssConfig cfg = track_cfg();
      cfg.align_yaw_min_var_m2 = floor_on ? 0.25 : 1e-9;
      GnssTrackYawAligner a(cfg);
      Line line;
      line.heading_rad = 0.2 * seed;
      for (int k = 0; k < 400; ++k) {
        const auto r = a.update(fix_at(line, k, kSigmaM, &rng), fix_cov(0.01), line.body(k), kPbg);
        if (r.is_ok()) {
          const double err = std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R)));
          beyond_10deg += err > 10.0 * kDeg ? 1 : 0;
          beyond_3s += err > 3.0 * a.yaw_sigma_rad() ? 1 : 0;
          break;
        }
      }
    }
    std::printf(
        "[ track yaw ] declared 1 cm, true 0.5 m, floor %s: %d of 100 > 10 deg, %d "
        "beyond 3 sigma\n",
        floor_on ? "0.25 m^2" : "off",
        beyond_10deg,
        beyond_3s);
    if (floor_on) {
      EXPECT_EQ(beyond_10deg, 0);
      EXPECT_LE(beyond_3s, 3);
    } else {
      EXPECT_GT(beyond_10deg, 20);  // negative control
    }
  }
}

// Review I3: a parked robot whose GNSS error wanders (random walk 0.05 m per fix plus 0.3 m
// white, declared 0.5 m) for 5 minutes. The sigma gate alone assumes white noise and aligned on
// 13 of 100 seeds (up to 180 deg off); with the 5 m minimum track no seed aligns here.
TEST(GnssTrackYaw, ParkedDriftingGnssNeverAligns) {
  GnssConfig cfg = track_cfg();
  cfg.align_yaw_min_track_m = 5.0;  // the default
  int n_aligned = 0;
  for (std::uint32_t seed = 1; seed <= 100; ++seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    GnssTrackYawAligner a(cfg);
    Line still;
    still.heading_rad = 0.1 * seed;
    still.static_fixes = 100000;
    Vec3 walk = Vec3::Zero();
    bool aligned = false;
    for (int k = 0; k < 1500 && !aligned; ++k) {
      Vec3 p = kTew * (still.body(k) * kPbg);
      for (int i = 0; i < 2; ++i) {
        walk(i) += 0.05 * n01(rng);
        p(i) += walk(i) + 0.3 * n01(rng);
      }
      aligned = a.update(p, fix_cov(kSigmaM), still.body(k), kPbg).is_ok();
    }
    n_aligned += aligned ? 1 : 0;
  }
  std::printf("[ track yaw ] parked, drifting GNSS: %d of 100 seeds aligned (coherence %.2f)\n",
              n_aligned,
              cfg.align_yaw_min_coherence);
  EXPECT_EQ(n_aligned, 0);
}

// Review I3: a declared covariance of zero made sigma_yaw zero (aligned at the second chunk,
// 45 of 100 runs > 10 deg off at 0.5 m true noise). Such fixes are now rejected and not added.
TEST(GnssTrackYaw, ZeroCovarianceNeverAligns) {
  std::mt19937 rng(9U);
  GnssTrackYawAligner a(track_cfg());
  Line line;
  for (int k = 0; k < 300; ++k) {
    const auto r = a.update(fix_at(line, k, kSigmaM, &rng), Mat3::Zero(), line.body(k), kPbg);
    ASSERT_FALSE(r.is_ok());
    EXPECT_EQ(r.reason(), "non-positive covariance");
  }
  EXPECT_TRUE(std::isinf(a.yaw_sigma_rad()));
}

// The minimum track (default 5 m) holds the transform back even when the declared noise is tiny.
TEST(GnssTrackYaw, MinTrackLength) {
  GnssConfig cfg = track_cfg();
  cfg.align_yaw_min_track_m = 5.0;
  GnssTrackYawAligner a(cfg);
  Line line;  // 0.1 m per fix
  int first = -1;
  for (int k = 0; k < 200 && first < 0; ++k) {
    const auto r = a.update(fix_at(line, k, 0.0, nullptr), fix_cov(1e-3), line.body(k), kPbg);
    if (r.is_ok()) {
      first = k;
    } else if (k >= 19) {
      EXPECT_EQ(r.reason(), "track too short") << k;
    }
  }
  // Chunk means 5 m apart need chunks 0 and 5: the transform comes at fix 59.
  EXPECT_EQ(first, 59);
  EXPECT_GE(a.track_m(), 5.0 - 1e-9);
}

TEST(GnssTrackYaw, NoAllocation) {
  GnssTrackYawAligner a(track_cfg());
  Line line;
  std::mt19937 rng(5U);
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int k = 0; k < 300; ++k) {
      (void)a.update(fix_at(line, k, kSigmaM, &rng), fix_cov(kSigmaM), line.body(k), kPbg);
    }
  }
  EXPECT_EQ(AllocCounter::count_this_thread(), 0U);
}

}  // namespace
