// Unit tests of the pipeline's GNSS fusion front (core/pipeline/src/gnss_fusion.hpp):
// pre-gate, chi-square gate, alignment and the fixes shifted to keyframe time
// (docs/derivations/pipeline.md section 5).
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/gnss/config.hpp"
#include "mss/gnss/gnss.hpp"
#include "mss/pipeline/bounded_queue.hpp"

#include "gnss_fusion.hpp"

namespace mss::pipeline::detail {
namespace {

using common::GnssMeasurement;
using common::Mat3;
using common::Pose;
using common::Timestamp;
using common::Vec3;

constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kSec = 1000000000;  // ns

const Vec3 kPbg(-0.20, 0.10, 0.60);  // m, lever arm in b

gnss::GnssConfig fusion_cfg() {
  gnss::GnssConfig c;
  c.enabled = true;
  c.sigma_h_m = 0.1;
  c.sigma_v_m = 0.2;
  c.max_cov_m2 = 25.0;
  return c;  // align_min_pairs 10, align_min_spread_m 1.0, chi2_gate 7.81 (defaults)
}

Pose truth_T_e_w() {
  Pose T;
  T.R = common::so3_exp(Vec3(0.0, 0.0, 30.0 * kPi / 180.0));
  T.p = Vec3(5.0, -3.0, 1.0);
  return T;
}

// Body on a 5 m circle, heading tangent.
Pose body_at(int k) {
  const double a = 0.5 * k;
  Pose T;
  T.R = common::so3_exp(Vec3(0.0, 0.0, a + 0.5 * kPi));
  T.p = Vec3(5.0 * std::cos(a), 5.0 * std::sin(a), 0.0);
  return T;
}

// Noise-free fix k at t = k s for body pose `T_w_b`, optionally offset in e.
GnssMeasurement fix_for(int k, const Pose& T_w_b, const Vec3& offset_e = Vec3::Zero()) {
  GnssMeasurement m;
  m.t = Timestamp{k * kSec};
  m.p_e_g = truth_T_e_w() * (T_w_b * kPbg) + offset_e;
  m.cov = Vec3(0.01, 0.01, 0.04).asDiagonal();
  m.status = 0;
  return m;
}

const common::PoseCovariance kZeroP = common::PoseCovariance::Zero();

// Feeds fixes 1..n (consistent); returns the number of ok statuses.
int align(GnssFusion& f, int n) {
  int n_ok = 0;
  for (int k = 1; k <= n; ++k) {
    n_ok += f.on_fix(fix_for(k, body_at(k)), body_at(k), kZeroP).is_ok() ? 1 : 0;
  }
  return n_ok;
}

TEST(GnssFusion, NotAlignedUntilMinPairs) {
  GnssFusion f(fusion_cfg(), kPbg);
  for (int k = 1; k <= 9; ++k) {
    EXPECT_EQ(f.on_fix(fix_for(k, body_at(k)), body_at(k), kZeroP).reason(), "gnss not aligned yet")
        << k;
    EXPECT_FALSE(f.aligned());
  }
  EXPECT_TRUE(f.on_fix(fix_for(10, body_at(10)), body_at(10), kZeroP).is_ok());
  EXPECT_TRUE(f.aligned());
  const Pose T = f.T_e_w();
  EXPECT_LT((T.p - truth_T_e_w().p).norm(), 1e-9);
  EXPECT_LT(common::so3_log(T.R.transpose() * truth_T_e_w().R).norm(), 1e-9);
}

TEST(GnssFusion, CollectShiftsFixToKeyframeTime) {
  GnssFusion f(fusion_cfg(), kPbg);
  ASSERT_EQ(align(f, 10), 1);  // fix 10 aligned and queued
  const Pose A = body_at(10);
  Pose B = A;
  B.p += Vec3(0.5, 0.0, 0.0);
  std::array<GnssMeasurement, kMaxAuxQueue> out{};
  ASSERT_EQ(f.collect(Timestamp{10 * kSec}, B, out), 1U);
  const Vec3 r = gnss::gnss_residual(out[0].p_e_g, f.T_e_w(), B, kPbg, nullptr);
  EXPECT_LT(r.norm(), 1e-9);
  EXPECT_TRUE(out[0].cov == fix_for(10, A).cov);
  // A later fix stays queued until a keyframe at or after its time.
  ASSERT_TRUE(f.on_fix(fix_for(11, body_at(11)), body_at(11), kZeroP).is_ok());
  EXPECT_EQ(f.collect(Timestamp{10 * kSec + kSec / 2}, B, out), 0U);
  EXPECT_EQ(f.collect(Timestamp{11 * kSec}, body_at(11), out), 1U);
  EXPECT_EQ(out[0].t, Timestamp{11 * kSec});
}

TEST(GnssFusion, JumpRejectedAfterAlignment) {  // Review Focus 5
  GnssFusion f(fusion_cfg(), kPbg);
  ASSERT_EQ(align(f, 20), 11);
  std::array<GnssMeasurement, kMaxAuxQueue> out{};
  (void)f.collect(Timestamp{20 * kSec}, body_at(20), out);
  const Pose before = f.T_e_w();
  const common::Status st =
      f.on_fix(fix_for(21, body_at(21), Vec3(5.0, 0.0, 0.0)), body_at(21), kZeroP);
  EXPECT_EQ(st.reason(), "chi-square gate exceeded");
  EXPECT_TRUE(f.T_e_w().R == before.R);
  EXPECT_TRUE(f.T_e_w().p == before.p);
  EXPECT_EQ(f.collect(Timestamp{100 * kSec}, body_at(21), out), 0U);
}

TEST(GnssFusion, NonFinitePoseCovTreatedAsZero) {  // Review Focus 5
  GnssFusion f(fusion_cfg(), kPbg);
  ASSERT_EQ(align(f, 10), 1);
  common::PoseCovariance nan_p;
  nan_p.setConstant(std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(f.on_fix(fix_for(11, body_at(11)), body_at(11), nan_p).is_ok());
  EXPECT_EQ(f.on_fix(fix_for(12, body_at(12), Vec3(5.0, 0.0, 0.0)), body_at(12), nan_p).reason(),
            "chi-square gate exceeded");
}

TEST(GnssFusion, GateFixReasonReturned) {
  GnssFusion f(fusion_cfg(), kPbg);
  GnssMeasurement m = fix_for(1, body_at(1));
  m.status = -1;
  EXPECT_EQ(f.on_fix(m, body_at(1), kZeroP).reason(), "no fix");
  m.status = 0;
  m.cov = Mat3::Identity() * 100.0;
  EXPECT_EQ(f.on_fix(m, body_at(1), kZeroP).reason(), "fix covariance too large");
}

TEST(GnssFusion, PendingBoundedDropsOldest) {
  GnssFusion f(fusion_cfg(), kPbg);
  const int extra = static_cast<int>(kMaxAuxQueue) + 4;
  ASSERT_EQ(align(f, 10 + extra), extra + 1);  // kMaxAuxQueue + 5 queued attempts
  EXPECT_EQ(f.n_pending_dropped(), 5U);
  std::array<GnssMeasurement, kMaxAuxQueue> out{};
  const std::size_t n = f.collect(Timestamp{100000 * kSec}, body_at(0), out);
  EXPECT_EQ(n, kMaxAuxQueue);
  EXPECT_EQ(out[0].t, Timestamp{15 * kSec});  // fixes 10..14 dropped
  EXPECT_EQ(out[n - 1].t, (Timestamp{static_cast<std::int64_t>(10 + extra) * kSec}));
}

}  // namespace
}  // namespace mss::pipeline::detail
