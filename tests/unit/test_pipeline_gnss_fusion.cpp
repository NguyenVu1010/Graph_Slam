// Unit tests of the pipeline's GNSS fusion front (core/pipeline/src/gnss_fusion.hpp):
// pre-gate, chi-square gate, alignment and the fixes shifted to keyframe time
// (docs/derivations/pipeline.md section 5), and the pre-alignment buffer (ADR-022).
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/gnss/config.hpp"
#include "mss/gnss/gnss.hpp"
#include "mss/pipeline/bounded_queue.hpp"

#include "alloc_counter.hpp"
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
  FixOutcome outcome = FixOutcome::kAccepted;
  const common::Status st =
      f.on_fix(fix_for(21, body_at(21), Vec3(5.0, 0.0, 0.0)), body_at(21), kZeroP, &outcome);
  EXPECT_EQ(st.reason(), "chi-square gate exceeded");
  EXPECT_EQ(outcome, FixOutcome::kChiSquare);
  EXPECT_TRUE(f.T_e_w().R == before.R);
  EXPECT_TRUE(f.T_e_w().p == before.p);
  EXPECT_EQ(f.collect(Timestamp{100 * kSec}, body_at(21), out), 0U);
}

// B-1 lock-out guard: N-1 consecutive chi-square rejections keep the alignment, the N-th
// restarts the aligner (that fix is its first pair) and factors flow again after
// align_min_pairs fixes consistent with the current front end.
TEST(GnssFusion, RealignsAfterMaxConsecutiveRejects) {
  gnss::GnssConfig c = fusion_cfg();
  c.max_consecutive_rejects = 3;
  GnssFusion f(c, kPbg);
  ASSERT_EQ(align(f, 20), 11);
  const Vec3 jump(5.0, 0.0, 0.0);  // m in e: the front end "drifted" by 5 m
  FixOutcome outcome = FixOutcome::kAccepted;
  for (int k = 21; k <= 22; ++k) {
    EXPECT_EQ(f.on_fix(fix_for(k, body_at(k), jump), body_at(k), kZeroP, &outcome).reason(),
              "chi-square gate exceeded");
    EXPECT_EQ(outcome, FixOutcome::kChiSquare);
    EXPECT_TRUE(f.aligned());
  }
  EXPECT_EQ(f.n_realign(), 0U);
  EXPECT_EQ(f.on_fix(fix_for(23, body_at(23), jump), body_at(23), kZeroP, &outcome).reason(),
            "gnss not aligned yet");
  EXPECT_EQ(outcome, FixOutcome::kNotAligned);
  EXPECT_FALSE(f.aligned());
  EXPECT_EQ(f.n_realign(), 1U);
  int n_ok = 0;
  for (int k = 24; k <= 40; ++k) {  // pairs 2..18 of the new alignment
    n_ok += f.on_fix(fix_for(k, body_at(k), jump), body_at(k), kZeroP).is_ok() ? 1 : 0;
  }
  EXPECT_EQ(n_ok, 17 - 8);  // the 10th pair (k = 32) and every later one
  EXPECT_TRUE(f.aligned());
  EXPECT_LT((f.T_e_w().p - (truth_T_e_w().p + jump)).norm(), 1e-9);
  EXPECT_EQ(f.n_realign(), 1U);
}

TEST(GnssFusion, AcceptedFixResetsRejectCount) {
  gnss::GnssConfig c = fusion_cfg();
  c.max_consecutive_rejects = 3;
  GnssFusion f(c, kPbg);
  ASSERT_EQ(align(f, 20), 11);
  const Vec3 jump(5.0, 0.0, 0.0);
  for (int k = 21; k <= 40; ++k) {  // reject, reject, accept, ...: never 3 in a row
    const bool bad = (k % 3) != 0;
    (void)f.on_fix(fix_for(k, body_at(k), bad ? jump : Vec3::Zero()), body_at(k), kZeroP);
  }
  EXPECT_EQ(f.n_realign(), 0U);
  EXPECT_TRUE(f.aligned());
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
  FixOutcome outcome = FixOutcome::kAccepted;
  EXPECT_EQ(f.on_fix(m, body_at(1), kZeroP, &outcome).reason(), "no fix");
  EXPECT_EQ(outcome, FixOutcome::kGateFix);
  m.status = 0;
  EXPECT_EQ(f.on_fix(m, body_at(1), kZeroP, &outcome).reason(), "gnss not aligned yet");
  EXPECT_EQ(outcome, FixOutcome::kNotAligned);
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

// ---------------------------------------------------------------- ADR-022 pre-alignment fixes

gnss::GnssConfig prealign_cfg(int max_fixes = 64) {
  gnss::GnssConfig c = fusion_cfg();
  c.attach_prealign_fixes = true;
  c.prealign_max_fixes = max_fixes;
  return c;
}

Timestamp at_s(int k) {
  return Timestamp{k * kSec};
}

// Residual of a released factor at its keyframe pose against the true T_e_w.
Vec3 residual_at(const EarlyFactor& e, const Pose& T_w_bk) {
  return e.m.p_e_g - truth_T_e_w() * (T_w_bk * kPbg);
}

// Flag off: the not-aligned fixes are dropped as before, nothing is buffered or released.
TEST(GnssFusionPrealign, OffKeepsNothing) {
  GnssFusion f(fusion_cfg(), kPbg);
  (void)f.on_preinit_fix(fix_for(0, body_at(0)));
  f.on_first_keyframe(0, at_s(0), body_at(0), kZeroP, at_s(-2), true);
  ASSERT_EQ(align(f, 10), 1);
  f.on_keyframe(1, at_s(10), body_at(10));
  std::array<EarlyFactor, kMaxAuxQueue> out{};
  EXPECT_EQ(f.take_prealign(out), 0U);
  EXPECT_EQ(f.n_prealign_buffered(), 0U);
  EXPECT_EQ(f.n_prealign_attached(), 0U);
  EXPECT_EQ(f.n_prealign_dropped(), 0U);
  EXPECT_EQ(f.n_prealign_not_static(), 0U);
}

// Tie rule: a fix exactly halfway between two keyframes goes to the earlier one; just after
// halfway, to the later one.
TEST(GnssFusionPrealign, TieGoesToEarlierKeyframe) {
  GnssFusion f(prealign_cfg(), kPbg);
  f.on_first_keyframe(0, at_s(0), body_at(0), kZeroP, at_s(-2), true);
  const Pose T = body_at(0);  // fixes consistent with this pose, so the gate passes
  GnssMeasurement half = fix_for(0, T);
  half.t = Timestamp{5 * kSec / 2};  // 2.5 s, halfway between 0 and 5 s
  GnssMeasurement after = fix_for(0, T);
  after.t = Timestamp{5 * kSec / 2 + 1};  // 1 ns later
  ASSERT_FALSE(f.on_fix(half, T, kZeroP).is_ok());
  ASSERT_FALSE(f.on_fix(after, T, kZeroP).is_ok());
  f.on_keyframe(1, at_s(5), body_at(5));
  for (int k = 6; k <= 13; ++k) {  // 8 more pairs: aligned at the 10th
    (void)f.on_fix(fix_for(k, body_at(k)), body_at(k), kZeroP);
  }
  ASSERT_TRUE(f.aligned());
  std::array<EarlyFactor, kMaxAuxQueue> out{};
  ASSERT_EQ(f.take_prealign(out), 2U);  // 6..12 wait for a keyframe after them
  EXPECT_EQ(out[0].m.t, half.t);
  EXPECT_EQ(out[0].kf, 0U);
  EXPECT_EQ(out[1].m.t, after.t);
  EXPECT_EQ(out[1].kf, 1U);
}

// Fixes 1..9 arrive before the alignment (fix 10). Keyframes 0 (t = 0), 1 (t = 5 s) and
// 2 (t = 10 s): each fix goes to the nearer one in time (a tie to the earlier one), shifted
// with the front-end poses, here exact, so each released factor has zero residual at its
// keyframe pose against the true T_e_w.
TEST(GnssFusionPrealign, AttachedToNearestKeyframeOnceAligned) {
  GnssFusion f(prealign_cfg(), kPbg);
  f.on_first_keyframe(0, at_s(0), body_at(0), kZeroP, at_s(-2), true);
  for (int k = 1; k <= 5; ++k) {
    ASSERT_FALSE(f.on_fix(fix_for(k, body_at(k)), body_at(k), kZeroP).is_ok());
  }
  f.on_keyframe(1, at_s(5), body_at(5));
  f.set_keyframe_reference(1, at_s(5), body_at(5));
  std::array<EarlyFactor, kMaxAuxQueue> out{};
  EXPECT_EQ(f.take_prealign(out), 0U);  // not aligned yet
  for (int k = 6; k <= 9; ++k) {
    ASSERT_FALSE(f.on_fix(fix_for(k, body_at(k)), body_at(k), kZeroP).is_ok());
  }
  ASSERT_TRUE(f.on_fix(fix_for(10, body_at(10)), body_at(10), kZeroP).is_ok());  // aligned
  EXPECT_EQ(f.n_prealign_buffered(), 9U);
  // Fixes 1..5 are assigned: 1, 2 to keyframe 0; 3, 4, 5 to keyframe 1. 6..9 wait.
  const std::array<common::KeyId, 5> kf_first = {0, 0, 1, 1, 1};
  ASSERT_EQ(f.take_prealign(out), 5U);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(out[i].m.t, at_s(static_cast<int>(i) + 1));
    EXPECT_EQ(out[i].kf, kf_first[i]) << i;
    const int kf_time = kf_first[i] == 0 ? 0 : 5;
    EXPECT_EQ(out[i].t_k, at_s(kf_time));
    EXPECT_LT(residual_at(out[i], body_at(kf_time)).norm(), 1e-9) << i;
    EXPECT_TRUE(out[i].m.cov == fix_for(1, body_at(1)).cov);  // cov unchanged
  }
  EXPECT_EQ(f.take_prealign(out), 0U);
  // Keyframe 2 at 10 s: 6, 7 to keyframe 1, 8, 9 to keyframe 2.
  f.on_keyframe(2, at_s(10), body_at(10));
  const std::array<common::KeyId, 4> kf_second = {1, 1, 2, 2};
  ASSERT_EQ(f.take_prealign(out), 4U);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(out[i].kf, kf_second[i]) << i;
    EXPECT_LT(residual_at(out[i], body_at(kf_second[i] == 1 ? 5 : 10)).norm(), 1e-9) << i;
  }
  EXPECT_EQ(f.take_prealign(out), 0U);
  EXPECT_EQ(f.n_prealign_attached(), 9U);
  EXPECT_EQ(f.n_prealign_buffered(), 0U);
  // The aligning fix itself still waits in the regular queue.
  std::array<GnssMeasurement, kMaxAuxQueue> queued{};
  EXPECT_EQ(f.collect(at_s(10), body_at(10), queued), 1U);
}

// Fixes received before keyframe 0: those of the static window [static_from, t_k] are
// attached to keyframe 0 unshifted; an older one and every one when the window is not known
// to be static are dropped and counted (n_prealign_not_static). A fix failing gate_fix
// returns its reason and is not kept. The chi-square gate runs at release: a 5 m jump is
// rejected and counted.
TEST(GnssFusionPrealign, StaticWindowFixesGoToKeyframeZero) {
  const Pose T0 = body_at(0);
  {
    GnssFusion f(prealign_cfg(), kPbg);
    for (int k = -3; k <= 0; ++k) {
      GnssMeasurement m = fix_for(0, T0);
      m.t = at_s(k);
      if (k == -1) {
        m.p_e_g += Vec3(5.0, 0.0, 0.0);
      }
      (void)f.on_preinit_fix(m);
    }
    GnssMeasurement late = fix_for(0, T0);
    late.t = Timestamp{kSec / 10};  // after keyframe 0's time
    EXPECT_TRUE(f.on_preinit_fix(late).is_ok());
    GnssMeasurement no_fix = fix_for(0, T0);
    no_fix.status = -1;
    EXPECT_EQ(f.on_preinit_fix(no_fix).reason(), "no fix");
    EXPECT_EQ(f.n_prealign_buffered(), 5U);
    f.on_first_keyframe(0, at_s(0), T0, kZeroP, at_s(-2), true);
    EXPECT_EQ(f.n_prealign_not_static(), 2U);  // t = -3 s and t = 0.1 s
    EXPECT_EQ(f.n_prealign_dropped(), 0U);
    EXPECT_EQ(f.n_prealign_buffered(), 3U);
    ASSERT_EQ(align(f, 10), 1);
    std::array<EarlyFactor, kMaxAuxQueue> out{};
    ASSERT_EQ(f.take_prealign(out), 2U);  // t = -2 s and t = 0; the -1 s jump rejected
    EXPECT_EQ(f.n_prealign_rejected(), 1U);
    EXPECT_EQ(f.n_prealign_static(), 2U);
    for (std::size_t i = 0; i < 2; ++i) {
      EXPECT_EQ(out[i].kf, 0U);
      EXPECT_EQ(out[i].t_k, at_s(0));
      EXPECT_LT(residual_at(out[i], T0).norm(), 1e-9);
    }
  }
  {
    GnssFusion f(prealign_cfg(), kPbg);
    (void)f.on_preinit_fix(fix_for(0, T0));
    f.on_first_keyframe(0, at_s(0), T0, kZeroP, at_s(-2), false);  // not the init scan
    EXPECT_EQ(f.n_prealign_not_static(), 1U);
    EXPECT_EQ(f.n_prealign_buffered(), 0U);
  }
}

// The buffer is bounded and reserved once: before keyframe 0 the oldest fix is dropped,
// after it the newest (the static ones stay); no allocation on any of these paths, the
// release (chi-square gate, shift, compaction) included.
TEST(GnssFusionPrealign, BufferBoundedNoAllocation) {
  GnssFusion f(prealign_cfg(4), kPbg);
  std::array<EarlyFactor, kMaxAuxQueue> out{};
  GnssMeasurement m = fix_for(0, body_at(0));
  std::size_t before_aligned = 1;
  std::size_t released = 0;
  std::size_t after_release = 1;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int k = -9; k <= 0; ++k) {
      m.t = at_s(k);
      (void)f.on_preinit_fix(m);
    }
    f.on_first_keyframe(0, at_s(0), body_at(0), kZeroP, at_s(-3), true);
    for (int k = 1; k <= 6; ++k) {
      (void)f.on_fix(fix_for(k, body_at(k)), body_at(k), kZeroP);
    }
    f.on_keyframe(1, at_s(6), body_at(6));
    before_aligned = f.take_prealign(out);
    for (int k = 7; k <= 10; ++k) {  // pairs 7..10: aligned at fix 10
      (void)f.on_fix(fix_for(k, body_at(k)), body_at(k), kZeroP);
    }
    f.on_keyframe(2, at_s(10), body_at(10));
    released = f.take_prealign(out);
    after_release = f.take_prealign(out);
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_EQ(before_aligned, 0U);  // not aligned yet: nothing released
  EXPECT_TRUE(f.aligned());
  // Pre-init: 10 fixes into 4 slots (6 oldest dropped), kept t = -3..0, all static, released
  // once aligned. After keyframe 0 the buffer is full: fixes 1..9 dropped.
  EXPECT_EQ(released, 4U);
  EXPECT_EQ(after_release, 0U);
  EXPECT_EQ(f.n_prealign_static(), 4U);
  EXPECT_EQ(f.n_prealign_dropped(), 15U);
  EXPECT_EQ(f.n_prealign_buffered(), 0U);
}

// A realign (gnss.max_consecutive_rejects) neither refills the buffer nor releases the early
// fixes a second time.
TEST(GnssFusionPrealign, RealignDoesNotAttachTwice) {
  gnss::GnssConfig c = prealign_cfg();
  c.max_consecutive_rejects = 3;
  GnssFusion f(c, kPbg);
  f.on_first_keyframe(0, at_s(0), body_at(0), kZeroP, at_s(-2), true);
  ASSERT_EQ(align(f, 10), 1);
  f.on_keyframe(1, at_s(10), body_at(10));
  std::array<EarlyFactor, kMaxAuxQueue> out{};
  ASSERT_EQ(f.take_prealign(out), 9U);
  EXPECT_EQ(f.take_prealign(out), 0U);
  const Vec3 jump(5.0, 0.0, 0.0);
  for (int k = 11; k <= 13; ++k) {  // the third rejection restarts the aligner
    (void)f.on_fix(fix_for(k, body_at(k), jump), body_at(k), kZeroP);
  }
  ASSERT_EQ(f.n_realign(), 1U);
  for (int k = 14; k <= 30; ++k) {  // realigned at k = 22
    (void)f.on_fix(fix_for(k, body_at(k), jump), body_at(k), kZeroP);
  }
  ASSERT_TRUE(f.aligned());
  EXPECT_EQ(f.n_prealign_buffered(), 0U);
  f.on_keyframe(2, at_s(30), body_at(30));
  EXPECT_EQ(f.take_prealign(out), 0U);
  EXPECT_EQ(f.n_prealign_attached(), 9U);
}

// ADR-023: gnss.align_yaw_from_track switches the fusion to GnssTrackYawAligner. Straight line
// at 0.5 m/s, 5 Hz, whose front-end along-axis positions run backwards (the corridor seeds 51 /
// 52 of gnss_fusion_study.md section 13): flag off flips by 180 deg, flag on does not, and the
// alignment counter and the predicted yaw sigma are reported.
TEST(GnssFusionTrackYaw, FlagSelectsTrackAligner) {
  for (const bool on : {false, true}) {
    gnss::GnssConfig c = fusion_cfg();
    c.align_yaw_from_track = on;
    GnssFusion f(c, kPbg);
    for (int k = 0; k < 150 && !f.aligned(); ++k) {
      Pose truth;
      truth.p = Vec3(0.1 * k, 0.0, 0.0);  // heading 0 (identity R)
      GnssMeasurement m;
      m.t = Timestamp{k * kSec / 5};
      m.p_e_g = truth_T_e_w() * (truth * kPbg);
      m.cov = Vec3(0.01, 0.01, 0.04).asDiagonal();
      m.status = 0;
      Pose fe = truth;
      fe.p.x() = -0.5 * truth.p.x();  // dead reckoning with the wrong sign
      (void)f.on_fix(m, fe, kZeroP);
    }
    ASSERT_TRUE(f.aligned()) << on;
    EXPECT_EQ(f.n_alignments(), 1U);
    const double err = std::abs(common::so3_log(f.T_e_w().R.transpose() * truth_T_e_w().R).z());
    if (on) {
      EXPECT_LT(err, 1e-6);  // noise-free fixes
      EXPECT_LE(f.align_yaw_sigma_rad(), c.align_yaw_max_sigma_rad);
    } else {
      EXPECT_GT(err, 3.0);  // about pi: the position fit flips
      // GnssAligner's sigma (backlog G1) takes the front end as exact: small, though 180 deg off.
      EXPECT_GT(f.align_yaw_sigma_rad(), 0.0);
      EXPECT_LT(f.align_yaw_sigma_rad(), 0.1);
    }
  }
}

// Backlog G1: with gnss.align_check_yaw_sigma the fusion passes the declared fix covariance to
// GnssAligner, which waits for its predicted yaw sigma: noise-free fixes on a 0.1 m-step line
// declared at 1 m^2, threshold 2 deg. Off: aligned at the 1 m spread (n = 35); on: once
// 1 / sqrt(0.01 (n^3 - n) / 12) <= 2 deg (n = 100), with that sigma reported.
TEST(GnssFusion, CheckYawSigmaUsesDeclaredCovariance) {
  for (const bool check : {false, true}) {
    gnss::GnssConfig c = fusion_cfg();
    c.align_check_yaw_sigma = check;
    c.align_yaw_max_sigma_rad = 2.0 * gnss::kRadPerDegree;
    GnssFusion f(c, kPbg);
    int n_aligned = 0;
    for (int k = 0; k < 200 && !f.aligned(); ++k) {
      Pose truth;
      truth.p = Vec3(0.1 * k, 0.0, 0.0);
      GnssMeasurement m;
      m.t = Timestamp{k * kSec / 5};
      m.p_e_g = truth_T_e_w() * (truth * kPbg);
      m.cov = Vec3(1.0, 1.0, 4.0).asDiagonal();
      m.status = 0;
      (void)f.on_fix(m, truth, kZeroP);
      n_aligned = k + 1;
    }
    ASSERT_TRUE(f.aligned()) << check;
    EXPECT_EQ(n_aligned, check ? 100 : 35) << check;
    const double sigma = f.align_yaw_sigma_rad();
    if (check) {
      EXPECT_LE(sigma, c.align_yaw_max_sigma_rad);
    } else {
      EXPECT_GT(sigma, c.align_yaw_max_sigma_rad);
    }
  }
}

// Backlog G15: GnssFusion passes the front-end covariance P_w_b of on_fix to GnssAligner: with
// gnss.align_yaw_sigma_frontend_cov and a front end that is 30 m uncertain along the line (1000
// m^2), the noise-free line of CheckYawSigmaUsesDeclaredCovariance never aligns within 200 fixes
// (threshold 2 deg); without the flag it aligns at n = 100 as there.
TEST(GnssFusion, FrontEndCovReachesTheAligner) {
  for (const bool flag : {false, true}) {
    gnss::GnssConfig c = fusion_cfg();
    c.align_check_yaw_sigma = true;
    c.align_yaw_max_sigma_rad = 2.0 * gnss::kRadPerDegree;
    c.align_yaw_sigma_frontend_cov = flag;
    GnssFusion f(c, kPbg);
    common::PoseCovariance P = common::PoseCovariance::Zero();
    P(0, 0) = 1000.0;
    int n_aligned = 0;
    for (int k = 0; k < 200 && !f.aligned(); ++k) {
      Pose truth;
      truth.p = Vec3(0.1 * k, 0.0, 0.0);
      GnssMeasurement m;
      m.t = Timestamp{k * kSec / 5};
      m.p_e_g = truth_T_e_w() * (truth * kPbg);
      m.cov = Vec3(1.0, 1.0, 4.0).asDiagonal();
      m.status = 0;
      (void)f.on_fix(m, truth, P);
      n_aligned = k + 1;
    }
    EXPECT_EQ(f.aligned(), !flag);
    if (!flag) {
      EXPECT_EQ(n_aligned, 100);
    }
  }
}

// ADR-023 addendum: GnssFusion::on_fix passes the wheel travel to the track aligner. A body
// driving backwards (x_b along +x, moving along -x) from the start: without wheel data the
// forward assumption gives 180 deg; with the wheel travel the yaw is right.
TEST(GnssFusionTrackYaw, WheelTravelGivesDirection) {
  for (const bool wheel : {false, true}) {
    gnss::GnssConfig c = fusion_cfg();
    c.align_yaw_from_track = true;
    GnssFusion f(c, kPbg);
    for (int k = 0; k < 150 && !f.aligned(); ++k) {
      Pose truth;
      truth.p = Vec3(-0.1 * k, 0.0, 0.0);  // heading 0, moving backwards
      GnssMeasurement m;
      m.t = Timestamp{k * kSec / 5};
      m.p_e_g = truth_T_e_w() * (truth * kPbg);
      m.cov = Vec3(0.01, 0.01, 0.04).asDiagonal();
      m.status = 0;
      const double odometer = wheel ? -0.1 * k : std::numeric_limits<double>::quiet_NaN();
      (void)f.on_fix(m, truth, kZeroP, nullptr, odometer);
    }
    ASSERT_TRUE(f.aligned()) << wheel;
    const double err = std::abs(common::so3_log(f.T_e_w().R.transpose() * truth_T_e_w().R).z());
    if (wheel) {
      EXPECT_LT(err, 1e-6);
      EXPECT_EQ(f.track_aligner().n_no_wheel_segments(), 0);
    } else {
      EXPECT_GT(err, 3.0);  // the documented fallback
      EXPECT_GT(f.track_aligner().n_no_wheel_segments(), 0);
    }
  }
}

// Re-review N3: the wheel travel of a fix rejected before the aligner (here by gate_fix, status
// < 0) is carried to the next fix, not lost. Every second fix is rejected while the body moves
// 0.015 m per fix: between two chunk centres (10 accepted fixes = 20 fixes) it travels 0.3 m,
// above the 0.2 m stationary threshold; counting only the accepted fixes' own steps would give
// 0.15 m and skip every segment as stationary.
TEST(GnssFusionTrackYaw, WheelTravelOfRejectedFixesCarried) {
  gnss::GnssConfig c = fusion_cfg();
  c.align_yaw_from_track = true;
  c.align_yaw_min_track_m = 1.0;
  GnssFusion f(c, kPbg);
  for (int k = 0; k < 600 && !f.aligned(); ++k) {
    Pose truth;
    truth.p = Vec3(0.015 * k, 0.0, 0.0);
    GnssMeasurement m;
    m.t = Timestamp{k * kSec / 5};
    m.p_e_g = truth_T_e_w() * (truth * kPbg);
    m.cov = Vec3(0.01, 0.01, 0.04).asDiagonal();
    m.status = k % 2 == 1 ? -1 : 0;  // odd fixes: "no fix", rejected by gate_fix
    (void)f.on_fix(m, truth, kZeroP, nullptr, 0.015 * k);
  }
  ASSERT_TRUE(f.aligned());
  EXPECT_EQ(f.track_aligner().n_stationary_segments(), 0);
  EXPECT_LT(std::abs(common::so3_log(f.T_e_w().R.transpose() * truth_T_e_w().R).z()), 1e-6);
}

// Re-review 2 coverage: a realign (gnss.max_consecutive_rejects) rebuilds the track-yaw aligner,
// whose odometer baseline restarts at NaN. Only the first fix after the realign loses its wheel
// travel, and it sits in the first half of the new first chunk, which no segment uses; every
// later travel is differenced from the absolute readings. The body drives backwards, so a lost
// travel would put a segment on the forward fallback and the yaw would flip.
TEST(GnssFusionTrackYaw, RealignKeepsWheelDifferencing) {
  gnss::GnssConfig c = fusion_cfg();
  c.align_yaw_from_track = true;
  c.align_yaw_min_track_m = 1.0;
  c.max_consecutive_rejects = 3;
  GnssFusion f(c, kPbg);
  const Vec3 jump(5.0, 0.0, 0.0);  // m, a frame offset the chi-square gate rejects
  int k = 0;
  const auto feed = [&](const Vec3& offset_e) {
    Pose truth;
    truth.p = Vec3(-0.1 * k, 0.0, 0.0);  // heading 0, moving backwards
    GnssMeasurement m;
    m.t = Timestamp{k * kSec / 5};
    m.p_e_g = truth_T_e_w() * (truth * kPbg) + offset_e;
    m.cov = Vec3(0.01, 0.01, 0.04).asDiagonal();
    m.status = 0;
    (void)f.on_fix(m, truth, kZeroP, nullptr, -0.1 * k);
    ++k;
  };
  while (!f.aligned() && k < 300) {
    feed(Vec3::Zero());
  }
  ASSERT_TRUE(f.aligned());
  EXPECT_EQ(f.n_alignments(), 1U);
  for (int i = 0; i < 3; ++i) {  // the third rejection restarts the aligner
    feed(jump);
  }
  ASSERT_EQ(f.n_realign(), 1U);
  EXPECT_FALSE(f.aligned());
  while (!f.aligned() && k < 600) {
    feed(jump);
  }
  ASSERT_TRUE(f.aligned());
  EXPECT_EQ(f.n_alignments(), 2U);
  EXPECT_EQ(f.track_aligner().n_no_wheel_segments(), 0);  // no travel lost after fix 1
  EXPECT_EQ(f.track_aligner().n_stationary_segments(), 0);
  EXPECT_GT(f.track_aligner().n_reversals(), 0);  // backward segments from the wheel sign
  EXPECT_LT(std::abs(common::so3_log(f.T_e_w().R.transpose() * truth_T_e_w().R).z()), 1e-6);
}

// Review R2-2 of design 2b: the heading variation that EstimatorCore passes to the track-yaw turn
// gate must be read from the travel axis x_o = R_w_b R_b_o x, as the aligner's heading. An IMU
// mounted with its x axis near vertical (R_b_o a pitch of 89.9 / 90 deg, x_o level) and 0.2 deg of
// attitude noise per scan: the heading of x_b is noise, its summed variation exceeds the turn gate
// in every chunk and the robot never aligned (0 of 50); read from x_o every run aligns, within its
// sigma. Straight track at 0.5 m/s, wheel data, 0.5 m fixes at 5 Hz, scans at 10 Hz, shipped keys.
TEST(GnssFusionTrackYaw, HeadingVariationUsesTheTravelAxis) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDeg = kPi / 180.0;
  gnss::GnssConfig c;
  c.align_yaw_from_track = true;
  c.align_yaw_max_sigma_rad = 6.0 * kDeg;
  c.align_yaw_wheel_turns = true;
  c.align_yaw_min_track_wheel_m = 3.0;
  const Vec3 p_b_g(-0.2, 0.1, 0.6);
  const auto rz = [](double yaw) { return Mat3(Eigen::AngleAxisd(yaw, Vec3::UnitZ())); };
  Pose T_e_w;
  T_e_w.R = rz(30.0 * kDeg);
  T_e_w.p = Vec3(12.0, -7.0, 1.0);
  for (const double tilt_deg : {89.9, 90.0}) {
    for (const bool from_x_o : {true, false}) {
      Pose T_b_o;
      T_b_o.R = Mat3(Eigen::AngleAxisd(tilt_deg * kDeg, Vec3::UnitY()));
      T_b_o.p = Vec3(-0.1, 0.05, 0.0);
      const Pose T_o_b = T_b_o.inverse();
      int aligned = 0;
      int beyond_3s = 0;
      for (std::uint32_t seed = 1; seed <= 50; ++seed) {
        std::mt19937 rng(7000U + seed);
        std::normal_distribution<double> n01(0.0, 1.0);
        gnss::GnssTrackYawAligner a(c, T_b_o);
        const double psi = 0.3 * seed;
        double variation = 0.0;
        double last_heading = 0.0;
        bool have_heading = false;
        Mat3 r_att = Mat3::Identity();
        for (int k = 0; k <= 1200; ++k) {  // 10 Hz scans, 120 s
          const double t = 0.1 * k;
          const double s = t < 2.0 ? 0.0 : 0.5 * (t - 2.0);
          Pose T_w_o;
          T_w_o.R = rz(psi);
          T_w_o.p = Vec3(s * std::cos(psi), s * std::sin(psi), 0.0);
          const Pose b = T_w_o * T_o_b;
          r_att = Mat3(Eigen::AngleAxisd(0.2 * kDeg * n01(rng), Vec3::UnitX()) *
                       Eigen::AngleAxisd(0.2 * kDeg * n01(rng), Vec3::UnitY()));
          const Mat3 r_meas = b.R * r_att;
          const double heading = from_x_o ? travel_heading_rad(r_meas, T_b_o.R)
                                          : std::atan2(r_meas(1, 0), r_meas(0, 0));
          if (have_heading) {
            variation += std::abs(std::remainder(heading - last_heading, 2.0 * kPi));
          }
          have_heading = true;
          last_heading = heading;
          if (k % 2 != 0) {
            continue;  // fixes at 5 Hz
          }
          Pose b_meas = b;
          b_meas.R = r_meas;
          Vec3 p = T_e_w * (b * p_b_g);
          p.x() += 0.5 * n01(rng);
          p.y() += 0.5 * n01(rng);
          const auto r = a.update(p, Mat3::Identity() * 0.25, b_meas, p_b_g, s, variation);
          if (r.is_ok()) {
            ++aligned;
            const double err = std::remainder(
                std::atan2(r.value().R(1, 0), r.value().R(0, 0)) - 30.0 * kDeg, 2.0 * kPi);
            beyond_3s += std::abs(err) > 3.0 * a.yaw_sigma_rad() ? 1 : 0;
            break;
          }
        }
      }
      if (from_x_o) {
        EXPECT_EQ(aligned, 50) << tilt_deg;
        EXPECT_LE(beyond_3s, 2) << tilt_deg;
      } else {
        EXPECT_LT(aligned, 10) << tilt_deg;  // negative control: the x_b heading is noise
      }
    }
  }
}

}  // namespace
// Backlog G3: the gate's measure is the marginal information along the travel axis (rotation
// and the other translations marginalised). Diagonal H: equal to the raw count. With rho_x (the
// travel axis in l) coupled to phi_z (a lever arm between the LiDAR and the surfaces): its Schur
// complement 400 - 150^2 / 90 = 150, while the raw count stays 400. Vertical travel axis: 0.
TEST(GnssFusionTrackYaw, TravelAxisMarginalInfo) {
  mss::common::Mat6 H = mss::common::Mat6::Zero();
  H(0, 0) = 400.0;
  H(1, 1) = 9.0;
  H(2, 2) = 50.0;
  H(3, 3) = 40.0;
  H(4, 4) = 60.0;
  H(5, 5) = 90.0;
  const Mat3 level = Mat3::Identity();
  EXPECT_NEAR(travel_axis_marginal_info(level, level, level, H), 400.0, 1e-9);
  const Mat3 yaw90 = Eigen::AngleAxisd(0.5 * kPi, Vec3::UnitZ()).toRotationMatrix();
  EXPECT_NEAR(travel_axis_marginal_info(level, yaw90, level, H), 9.0, 1e-9);
  H(0, 5) = 150.0;
  H(5, 0) = 150.0;
  EXPECT_NEAR(travel_axis_marginal_info(level, level, level, H), 150.0, 1e-9);
  EXPECT_NEAR(travel_axis_info(level, level, level, H), 400.0, 1e-9);
  const Mat3 pitch90 = Eigen::AngleAxisd(0.5 * kPi, Vec3::UnitY()).toRotationMatrix();
  EXPECT_EQ(travel_axis_marginal_info(level, pitch90, level, H), 0.0);
  // The gate: observed at a threshold of 150, not at 200, although the raw count is 400.
  const Vec3 no_arm = Vec3::Zero();
  EXPECT_TRUE(travel_axis_observed(level, level, level, H, no_arm, 149.9));
  EXPECT_FALSE(travel_axis_observed(level, level, level, H, no_arm, 200.0));
  // No information never passes, also at a threshold of 0 (H4 review I-2): a vertical travel
  // axis, and an axis along a null direction of H.
  EXPECT_FALSE(travel_axis_observed(level, pitch90, level, H, no_arm, 0.0));
  mss::common::Mat6 corridor = H;
  corridor.row(0).setZero();
  corridor.col(0).setZero();
  EXPECT_EQ(travel_axis_marginal_info(level, level, level, corridor), 0.0);
  EXPECT_FALSE(travel_axis_observed(level, level, level, corridor, no_arm, 0.0));
}

// H4 review m-2: the gate measures the odometry frame's origin o, not the LiDAR's. With o 1 m
// below the LiDAR (p_l_o = (0, 0, -1)) and the travel axis along l x, a pitch error moves o along
// the axis: u = [x; p_l_o x x] = [x; (0, -1, 0)], so with no coupling in H the information is
// 1 / (1/400 + 1/60) = 52.17, against 400 at the LiDAR origin.
TEST(GnssFusionTrackYaw, TravelAxisMarginalInfoOfTheOdometryFrame) {
  mss::common::Mat6 H = mss::common::Mat6::Zero();
  H.diagonal() << 400.0, 9.0, 50.0, 40.0, 60.0, 90.0;
  const Mat3 level = Mat3::Identity();
  EXPECT_NEAR(travel_axis_marginal_info(level, level, level, H, Vec3::Zero()), 400.0, 1e-9);
  const double expected = 1.0 / (1.0 / 400.0 + 1.0 / 60.0);
  EXPECT_NEAR(
      travel_axis_marginal_info(level, level, level, H, Vec3(0.0, 0.0, -1.0)), expected, 1e-9);
  EXPECT_FALSE(travel_axis_observed(level, level, level, H, Vec3(0.0, 0.0, -1.0), 100.0));
}

// Root-cause study: the information along the horizontal travel axis, d^T H_rho d with d in the
// LiDAR frame. H_rho = diag(400, 9, 50): with l = b = o and a yaw of 90 deg, x_o points along w y,
// which is l x, so 400; with T_b_o turned by 90 deg about z it is l y, so 9; with x_o pitched to
// vertical its horizontal part is undefined: 0. A tilted x_o uses its horizontal direction.
TEST(GnssFusionTrackYaw, TravelAxisInfo) {
  mss::common::Mat6 H = mss::common::Mat6::Zero();
  H(0, 0) = 400.0;
  H(1, 1) = 9.0;
  H(2, 2) = 50.0;
  H(3, 3) = 1e6;  // rotation block: not read
  const Mat3 R_w_b = Eigen::AngleAxisd(0.5 * kPi, Vec3::UnitZ()).toRotationMatrix();
  const Mat3 R_w_l = R_w_b;
  EXPECT_NEAR(travel_axis_info(R_w_b, Mat3::Identity(), R_w_l, H), 400.0, 1e-9);
  const Mat3 yaw90 = Eigen::AngleAxisd(0.5 * kPi, Vec3::UnitZ()).toRotationMatrix();
  EXPECT_NEAR(travel_axis_info(R_w_b, yaw90, R_w_l, H), 9.0, 1e-9);
  const Mat3 pitch90 = Eigen::AngleAxisd(0.5 * kPi, Vec3::UnitY()).toRotationMatrix();
  const Mat3 level = Mat3::Identity();
  EXPECT_EQ(travel_axis_info(level, pitch90, level, H), 0.0);
  const Mat3 pitch30 = Eigen::AngleAxisd(30.0 * kPi / 180.0, Vec3::UnitY()).toRotationMatrix();
  EXPECT_NEAR(travel_axis_info(level, pitch30, level, H), 400.0, 1e-9);
  // The axis goes into l with R_w_l^T: x_o along w x, l yawed by 45 deg, so d_l = (c, -s, 0) with
  // c = s = 1/sqrt(2); with H(0, 1) = 100: 0.5 400 + 0.5 9 - 100 = 104.5.
  H(0, 1) = 100.0;
  H(1, 0) = 100.0;
  const Mat3 yaw45 = Eigen::AngleAxisd(0.25 * kPi, Vec3::UnitZ()).toRotationMatrix();
  EXPECT_NEAR(travel_axis_info(level, Mat3::Identity(), yaw45, H), 104.5, 1e-9);
}

}  // namespace mss::pipeline::detail
