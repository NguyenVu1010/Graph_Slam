// Unit tests of the pipeline's GNSS fusion front (core/pipeline/src/gnss_fusion.hpp):
// pre-gate, chi-square gate, alignment and the fixes shifted to keyframe time
// (docs/derivations/pipeline.md section 5), and the pre-alignment buffer (ADR-022).
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
      EXPECT_TRUE(std::isnan(f.align_yaw_sigma_rad()));
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

}  // namespace
}  // namespace mss::pipeline::detail
