// L-01..L-03 (Test_CI_CD.md, loop module) and the LoopDetector contract (ADR-026), with the
// correction gate from the graph's uncertainty and the ambiguity check (ADR-026 amendment).
// Scenes: tests/unit/loop_scenes.hpp (no randomness; every value fixed).
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/span.hpp"
#include "mss/loop/loop_detector.hpp"

#include "loop_scenes.hpp"

namespace mss::loop {
namespace {

using common::Pose;
using common::ScanFrame;
using test::body_pose;
using test::kLoopPi;

constexpr double kDeg = kLoopPi / 180.0;
constexpr std::size_t kMaxPoints = 40000;
constexpr double kRangeM = 15.0;             // scan range of the test "LiDAR", m
constexpr std::int64_t kNs = 1'000'000'000;  // ns per s
// Measurement accuracy asked of an accepted loop on these noise-free scenes.
constexpr double kMaxLoopErrM = 0.05;
constexpr double kMaxLoopErrRad = 0.5 * kDeg;

double yaw(const common::Mat3& R) {
  return std::atan2(R(1, 0), R(0, 0));
}

double wrap(double a) {
  return std::remainder(a, 2.0 * kLoopPi);
}

// The graph's relative covariance as a fixed matrix (or an error), for the correction gate.
class FixedRelativeCovariance : public RelativeCovarianceSource {
 public:
  explicit FixedRelativeCovariance(const common::PoseCovariance& c) : c_(c) {}
  common::Result<common::PoseCovariance> relative_cov(common::KeyId /*i*/,
                                                      common::KeyId /*j*/) const override {
    ++calls;
    if (fail) {
      return common::Result<common::PoseCovariance>::error("test: no marginal");
    }
    return common::Result<common::PoseCovariance>::ok(c_);
  }
  bool fail = false;
  mutable int calls = 0;

 private:
  common::PoseCovariance c_;
};

common::PoseCovariance diag_cov(double sig_m, double sig_rad) {
  common::Vec6 d;
  d << sig_m, sig_m, sig_m, sig_rad, sig_rad, sig_rad;
  const common::PoseCovariance c = d.cwiseAbs2().asDiagonal();
  return c;
}

struct Pair {
  Pose truth_c;  // true body poses
  Pose truth_q;
  Pose graph_c;  // poses the "backend" reports (drifted)
  Pose graph_q;
};

class LoopFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    cfg_ = test::shipped_loop_config();
    ASSERT_TRUE(cfg_.loop_closure.enabled) << "config/default.yaml did not load";
  }

  LoopResult run(const std::vector<common::Vec3>& world_c,
                 const std::vector<common::Vec3>& world_q,
                 const Pair& pr,
                 const LoopConfig& lc,
                 const RelativeCovarianceSource* rel = nullptr) {
    const Pose T_b_l = test::loop_T_b_l();
    test::scan_at(world_c, pr.truth_c * T_b_l, kRangeM, cloud_c_);
    test::scan_at(world_q, pr.truth_q * T_b_l, kRangeM, cloud_q_);
    // A source here means the optional graph gate (graph_covariance_gate, ADR-028) is on.
    LoopConfig gated = lc;
    gated.graph_covariance_gate = gated.graph_covariance_gate || rel != nullptr;
    LoopDetector det(gated, cfg_.scan_match, cfg_.map, T_b_l, kMaxPoints);
    std::vector<KeyframeView> kfs(2);
    kfs[0].id = 0;
    kfs[0].t = common::Timestamp{0};
    kfs[0].T_w_b = pr.graph_c;
    kfs[0].cloud_l = &cloud_c_;
    kfs[1].id = 1;
    kfs[1].t = common::Timestamp{100 * kNs};
    kfs[1].T_w_b = pr.graph_q;
    kfs[1].cloud_l = &cloud_q_;
    return det.detect(kfs[1], common::Span<const KeyframeView>(kfs), rel);
  }

  // |measured T_bc_bq - true T_bc_bq|: translation (m) and yaw (rad).
  static void loop_error(const LoopResult& r, const Pair& pr, double& e_p, double& e_yaw) {
    const Pose truth = pr.truth_c.inverse() * pr.truth_q;
    e_p = (r.constraint.T_bi_bj.p - truth.p).norm();
    e_yaw = std::abs(wrap(yaw(r.constraint.T_bi_bj.R) - yaw(truth.R)));
  }

  pipeline::Config cfg_;
  ScanFrame cloud_c_{kMaxPoints};
  ScanFrame cloud_q_{kMaxPoints};
};

// L-01: same place, heading 90 deg apart. The graph pose of the query carries drift (0.4 m,
// and 0 / 10 / 20 deg of yaw). Accepted; the Scan Context yaw is within one sector of the true
// relative yaw; the measurement matches the truth.
TEST_F(LoopFixture, L01_SamePlaceDifferentHeading) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  const double sector = 2.0 * kLoopPi / cfg_.loop_closure.sc_sectors;
  for (const double drift_deg : {0.0, 10.0, 20.0}) {
    Pair pr;
    pr.truth_c = body_pose(-2.0, 1.0, 0.0);
    pr.truth_q = body_pose(-1.8, 1.1, 90.0 * kDeg);
    pr.graph_c = pr.truth_c;
    pr.graph_q = body_pose(-1.4, 1.1, (90.0 + drift_deg) * kDeg);
    const LoopResult r = run(hall, hall, pr, cfg_.loop_closure);
    ASSERT_EQ(r.outcome, LoopOutcome::kAccepted)
        << "drift " << drift_deg << " deg: outcome " << static_cast<int>(r.outcome) << " sc "
        << r.sc_distance << " ratio " << r.inlier_ratio << " rms " << r.rms_m << " eig "
        << r.min_eig;
    EXPECT_LE(std::abs(wrap(r.sc_yaw_rad - 90.0 * kDeg)), sector) << r.sc_yaw_rad / kDeg;
    double e_p = 0.0;
    double e_yaw = 0.0;
    loop_error(r, pr, e_p, e_yaw);
    EXPECT_LT(e_p, kMaxLoopErrM) << "drift " << drift_deg;
    EXPECT_LT(e_yaw, kMaxLoopErrRad) << "drift " << drift_deg;
    EXPECT_EQ(r.constraint.i, 0U);
    EXPECT_EQ(r.constraint.j, 1U);
  }
}

// L-02: precision. Pairs at truly different places (4 to 12 m apart) whose graph poses say
// "same place" (drift), plus pairs at the same place: no accepted loop may disagree with the
// truth. Recall of the same-place pairs is reported.
TEST_F(LoopFixture, L02_DifferentPlacesNoFalseLoop) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  const double places[][2] = {
      {-14.0, 0.0}, {-8.0, -2.0}, {-3.0, 3.0}, {3.0, 0.5}, {9.0, -1.0}, {14.0, 3.0}};
  const int n = static_cast<int>(sizeof(places) / sizeof(places[0]));
  int accepted = 0;
  int same_place_accepted = 0;
  int same_place = 0;
  for (int a = 0; a < n; ++a) {
    for (int b = 0; b < n; ++b) {
      const double yaw_c = 30.0 * a * kDeg;
      const double yaw_q = 45.0 * b * kDeg;
      Pair pr;
      pr.truth_c = body_pose(places[a][0], places[a][1], yaw_c);
      pr.truth_q = body_pose(places[b][0] + 0.3, places[b][1], yaw_q);
      pr.graph_c = pr.truth_c;
      // The graph believes q is next to c whatever the truth (drift of up to 12 m).
      pr.graph_q = body_pose(places[a][0] + 0.3, places[a][1], yaw_q);
      const LoopResult r = run(hall, hall, pr, cfg_.loop_closure);
      same_place += a == b ? 1 : 0;
      if (r.outcome != LoopOutcome::kAccepted) {
        continue;
      }
      ++accepted;
      same_place_accepted += a == b ? 1 : 0;
      double e_p = 0.0;
      double e_yaw = 0.0;
      loop_error(r, pr, e_p, e_yaw);
      EXPECT_LT(e_p, kMaxLoopErrM) << "false loop c " << a << " q " << b << " sc " << r.sc_distance
                                   << " ratio " << r.inlier_ratio << " rms " << r.rms_m;
      EXPECT_LT(e_yaw, kMaxLoopErrRad) << "false loop c " << a << " q " << b;
    }
  }
  RecordProperty("accepted", accepted);
  RecordProperty("same_place_recall", same_place_accepted);
  std::printf("L-02: %d accepted of %d pairs, same-place recall %d / %d\n",
              accepted,
              n * n,
              same_place_accepted,
              same_place);
  EXPECT_GE(same_place_accepted, same_place / 2);  // the detector does find loops
}

// L-03: a deliberately wrong candidate (another place 6 m away, the graph says the same place,
// Scan Context and correction gates disabled so ICP runs and converges where it can): the
// fitness gates (inlier ratio, RMS) reject it.
TEST_F(LoopFixture, L03_WrongCandidateRejectedByFitness) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  LoopConfig lc = cfg_.loop_closure;
  lc.sc_max_distance = 1.0;
  lc.max_correction_m = 1e3;
  lc.max_correction_rad = kLoopPi;
  Pair pr;
  pr.truth_c = body_pose(-9.0, -2.0, 0.0);
  pr.truth_q = body_pose(-3.0, 3.0, 0.0);
  pr.graph_c = pr.truth_c;
  pr.graph_q = body_pose(-9.0, -2.0, 0.0);
  const LoopResult r = run(hall, hall, pr, lc);
  EXPECT_EQ(r.n_verified, 1);
  EXPECT_TRUE(r.outcome == LoopOutcome::kInliers || r.outcome == LoopOutcome::kRms)
      << "outcome " << static_cast<int>(r.outcome) << " ratio " << r.inlier_ratio << " rms "
      << r.rms_m;
}

// Degeneracy gate: a corridor without an end wall in range; the same place, the graph 0.5 m
// off along the corridor. ICP cannot observe that axis: rejected as degenerate.
TEST_F(LoopFixture, CorridorRejectedAsDegenerate) {
  const std::vector<common::Vec3> corr = test::corridor_w();
  Pair pr;
  pr.truth_c = body_pose(0.0, 0.0, 0.0);
  pr.truth_q = body_pose(0.2, 0.1, 5.0 * kDeg);
  pr.graph_c = pr.truth_c;
  pr.graph_q = body_pose(0.7, 0.1, 5.0 * kDeg);
  const LoopResult r = run(corr, corr, pr, cfg_.loop_closure);
  EXPECT_EQ(r.outcome, LoopOutcome::kDegenerate)
      << "outcome " << static_cast<int>(r.outcome) << " eig " << r.min_eig;
}

// The loop covariance is in b_q, symmetric and floored: C - diag(floor^2) is PSD.
TEST_F(LoopFixture, CovarianceFloored) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  Pair pr;
  pr.truth_c = body_pose(3.0, 0.5, 0.0);
  pr.truth_q = body_pose(3.2, 0.5, 10.0 * kDeg);
  pr.graph_c = pr.truth_c;
  pr.graph_q = pr.truth_q;
  const LoopResult r = run(hall, hall, pr, cfg_.loop_closure);
  ASSERT_EQ(r.outcome, LoopOutcome::kAccepted);
  const common::Mat6& C = r.constraint.cov;
  EXPECT_LT((C - C.transpose()).cwiseAbs().maxCoeff(), 1e-15);
  common::Vec6 d;
  const double fm = cfg_.loop_closure.cov_floor_m;
  const double fr = cfg_.loop_closure.cov_floor_rad;
  d << fm, fm, fm, fr, fr, fr;
  const common::Mat6 W = d.cwiseInverse().asDiagonal() * C * d.cwiseInverse().asDiagonal();
  const Eigen::SelfAdjointEigenSolver<common::Mat6> eig(W);
  EXPECT_GE(eig.eigenvalues()(0), 1.0 - 1e-9);
}

// No candidate: too recent (time gap) or too far (radius); the query is never its own
// candidate.
TEST_F(LoopFixture, CandidatesNeedTimeGapAndRadius) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  const Pose T_b_l = test::loop_T_b_l();
  test::scan_at(hall, body_pose(0.0, 0.0, 0.0) * T_b_l, kRangeM, cloud_c_);
  LoopDetector det(cfg_.loop_closure, cfg_.scan_match, cfg_.map, T_b_l, kMaxPoints);
  std::vector<KeyframeView> kfs(2);
  kfs[0].id = 0;
  kfs[0].t = common::Timestamp{0};
  kfs[0].T_w_b = body_pose(0.0, 0.0, 0.0);
  kfs[0].cloud_l = &cloud_c_;
  kfs[1] = kfs[0];
  kfs[1].id = 1;
  const std::int64_t gap_ns =
      static_cast<std::int64_t>(std::llround(cfg_.loop_closure.min_time_gap_s * 1e9));
  kfs[1].t = common::Timestamp{gap_ns - 1};
  EXPECT_EQ(det.detect(kfs[1], common::Span<const KeyframeView>(kfs)).outcome,
            LoopOutcome::kNoCandidate);
  kfs[1].t = common::Timestamp{gap_ns};
  kfs[1].T_w_b = body_pose(cfg_.loop_closure.search_radius_m + 0.1, 0.0, 0.0);
  EXPECT_EQ(det.detect(kfs[1], common::Span<const KeyframeView>(kfs)).outcome,
            LoopOutcome::kNoCandidate);
  kfs[1].T_w_b = body_pose(0.1, 0.0, 0.0);
  const LoopResult r = det.detect(kfs[1], common::Span<const KeyframeView>(kfs));
  EXPECT_EQ(r.n_candidates, 1);
  EXPECT_EQ(r.outcome, LoopOutcome::kAccepted);
}

// Scan Context: the same cloud with the body turned by k sectors gives shift k (relative yaw
// k sectors) and distance ~0.
TEST_F(LoopFixture, ScanContextShiftIsRelativeYaw) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  const Pose T_b_l = test::loop_T_b_l();
  LoopDetector det(cfg_.loop_closure, cfg_.scan_match, cfg_.map, T_b_l, kMaxPoints);
  const int sectors = cfg_.loop_closure.sc_sectors;
  const double sector = 2.0 * kLoopPi / sectors;
  std::vector<float> dc;
  std::vector<float> dq;
  // Bodies on the same spot (the LiDAR is 0.1 m ahead of the body, so the scans differ
  // slightly), yaw 0 and 7 sectors (+0.4 sector to stay off a bin edge).
  const Pose bc = body_pose(1.0, 0.0, 0.0);
  const Pose bq = body_pose(1.0, 0.0, 7.4 * sector);
  test::scan_at(hall, bc * T_b_l, kRangeM, cloud_c_);
  test::scan_at(hall, bq * T_b_l, kRangeM, cloud_q_);
  det.descriptor(cloud_c_, bc, dc);
  det.descriptor(cloud_q_, bq, dq);
  int shift = -1;
  const double d = scan_context_distance(common::Span<const float>(dq),
                                         common::Span<const float>(dc),
                                         cfg_.loop_closure.sc_rings,
                                         sectors,
                                         shift);
  EXPECT_LE(std::abs(wrap(shift * sector - 7.4 * sector)), sector) << shift;
  EXPECT_LT(d, cfg_.loop_closure.sc_max_distance);
}

// --- Correction gate from the graph's uncertainty (ADR-026 amendment, C1-12, C1-16) --------

// The same 0.4 m correction is accepted when the graph's relative covariance allows it
// (sigma 0.5 m / 5 deg) and rejected when it does not (1 cm / 0.1 deg: e^T S^-1 e of a 0.4 m
// correction against S ~ (5 cm floor)^2 is ~64 > 22.46). The marginal is asked once.
TEST_F(LoopFixture, CorrectionGateFollowsGraphCovariance) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  Pair pr;
  pr.truth_c = body_pose(3.0, 0.5, 0.0);
  pr.truth_q = body_pose(3.2, 0.5, 10.0 * kDeg);
  pr.graph_c = pr.truth_c;
  pr.graph_q = body_pose(3.6, 0.5, 10.0 * kDeg);
  const FixedRelativeCovariance loose(diag_cov(0.5, 5.0 * kDeg));
  const LoopResult ok = run(hall, hall, pr, cfg_.loop_closure, &loose);
  ASSERT_EQ(ok.outcome, LoopOutcome::kAccepted) << static_cast<int>(ok.outcome);
  EXPECT_LT(ok.correction_chi2, 2.0);
  EXPECT_EQ(loose.calls, 1);
  double e_p = 0.0;
  double e_yaw = 0.0;
  loop_error(ok, pr, e_p, e_yaw);
  EXPECT_LT(e_p, kMaxLoopErrM);
  const FixedRelativeCovariance tight(diag_cov(0.01, 0.1 * kDeg));
  const LoopResult no = run(hall, hall, pr, cfg_.loop_closure, &tight);
  EXPECT_EQ(no.outcome, LoopOutcome::kCorrection) << static_cast<int>(no.outcome);
  EXPECT_GT(no.correction_chi2, cfg_.loop_closure.correction_chi2_gate);
  // Without a source only the fixed 3 m / 30 deg gates apply: accepted.
  EXPECT_EQ(run(hall, hall, pr, cfg_.loop_closure).outcome, LoopOutcome::kAccepted);
  // ADR-028: with graph_covariance_gate off (shipped) a source is ignored: accepted, no marginal
  // asked.
  ASSERT_FALSE(cfg_.loop_closure.graph_covariance_gate);
  const Pose T_b_l = test::loop_T_b_l();
  test::scan_at(hall, pr.truth_c * T_b_l, kRangeM, cloud_c_);
  test::scan_at(hall, pr.truth_q * T_b_l, kRangeM, cloud_q_);
  LoopDetector det(cfg_.loop_closure, cfg_.scan_match, cfg_.map, T_b_l, kMaxPoints);
  std::vector<KeyframeView> kfs(2);
  kfs[0].id = 0;
  kfs[0].T_w_b = pr.graph_c;
  kfs[0].cloud_l = &cloud_c_;
  kfs[1].id = 1;
  kfs[1].t = common::Timestamp{100 * kNs};
  kfs[1].T_w_b = pr.graph_q;
  kfs[1].cloud_l = &cloud_q_;
  const FixedRelativeCovariance tight2(diag_cov(0.01, 0.1 * kDeg));
  const LoopResult off = det.detect(kfs[1], common::Span<const KeyframeView>(kfs), &tight2);
  EXPECT_EQ(off.outcome, LoopOutcome::kAccepted);
  EXPECT_EQ(tight2.calls, 0);
  EXPECT_TRUE(std::isnan(off.correction_chi2));
}

// C1-16: the gate covers roll and pitch. The graph has the query tilted 6 deg in roll (the
// truth is level) and is confident about it; ICP levels it. The fixed gates (position, yaw)
// let the 6 deg roll correction through; the chi-square gate rejects it (~(6 / 1 deg floor)^2).
TEST_F(LoopFixture, CorrectionGateCoversRollAndPitch) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  Pair pr;
  pr.truth_c = body_pose(3.0, 0.5, 0.0);
  pr.truth_q = body_pose(3.2, 0.5, 10.0 * kDeg);
  pr.graph_c = pr.truth_c;
  for (const int axis : {0, 1}) {
    common::Vec3 phi = common::Vec3::Zero();
    phi(axis) = 6.0 * kDeg;
    pr.graph_q = pr.truth_q;
    pr.graph_q.R = pr.truth_q.R * common::so3_exp(phi);
    const LoopResult fixed_only = run(hall, hall, pr, cfg_.loop_closure);
    ASSERT_EQ(fixed_only.outcome, LoopOutcome::kAccepted) << "axis " << axis;
    const FixedRelativeCovariance tight(diag_cov(0.01, 0.1 * kDeg));
    const LoopResult r = run(hall, hall, pr, cfg_.loop_closure, &tight);
    EXPECT_EQ(r.outcome, LoopOutcome::kCorrection) << "axis " << axis;
    EXPECT_GT(r.correction_chi2, cfg_.loop_closure.correction_chi2_gate) << "axis " << axis;
  }
}

// A source that cannot give the marginal rejects the candidate (fail closed).
TEST_F(LoopFixture, CorrectionGateFailsClosedWithoutMarginal) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  Pair pr;
  pr.truth_c = body_pose(3.0, 0.5, 0.0);
  pr.truth_q = body_pose(3.2, 0.5, 10.0 * kDeg);
  pr.graph_c = pr.truth_c;
  pr.graph_q = pr.truth_q;
  FixedRelativeCovariance broken(diag_cov(0.5, 5.0 * kDeg));
  broken.fail = true;
  const LoopResult r = run(hall, hall, pr, cfg_.loop_closure, &broken);
  EXPECT_EQ(r.outcome, LoopOutcome::kNoMarginal);
  EXPECT_EQ(broken.calls, 1);
}

// --- Ambiguity check (C1-12) ----------------------------------------------------------------

// Pillar pairs every P m along a corridor: without a graph covariance the detector cannot tell
// the true fit from the copy one period away (inlier ratio 0.93-0.94 of the true one), so it
// rejects the loop whatever the drift, instead of accepting a false loop above P/2.
TEST_F(LoopFixture, AmbiguityRejectsPeriodicStructure) {
  for (const double period : {2.0, 2.5}) {
    const std::vector<common::Vec3> w = test::periodic_corridor_w(period);
    for (const double drift : {0.0, 0.6, 1.3, 2.0}) {
      Pair pr;
      pr.truth_c = body_pose(0.0, 0.0, 0.0);
      pr.truth_q = body_pose(0.1, 0.1, 3.0 * kDeg);
      pr.graph_c = pr.truth_c;
      pr.graph_q = body_pose(0.1 + drift, 0.1, 3.0 * kDeg);
      const LoopResult r = run(w, w, pr, cfg_.loop_closure);
      EXPECT_EQ(r.outcome, LoopOutcome::kAmbiguous)
          << "P " << period << " drift " << drift << ": outcome " << static_cast<int>(r.outcome);
      EXPECT_GE(r.ambiguity_ratio, cfg_.loop_closure.ambiguity_max_ratio);
    }
  }
  // With the check off the false loop of the review comes back (drift 1.3 m, P = 2 m).
  LoopConfig off = cfg_.loop_closure;
  off.ambiguity_seeds_per_side = 0;
  Pair pr;
  pr.truth_c = body_pose(0.0, 0.0, 0.0);
  pr.truth_q = body_pose(0.1, 0.1, 3.0 * kDeg);
  pr.graph_c = pr.truth_c;
  pr.graph_q = body_pose(1.4, 0.1, 3.0 * kDeg);
  const LoopResult r = run(test::periodic_corridor_w(2.0), test::periodic_corridor_w(2.0), pr, off);
  ASSERT_EQ(r.outcome, LoopOutcome::kAccepted);
  double e_p = 0.0;
  double e_yaw = 0.0;
  loop_error(r, pr, e_p, e_yaw);
  EXPECT_GT(e_p, 1.5);
}

// In the irregular hall the shifted seeds find no competitor: L-01's places are accepted with
// the ambiguity check on, and every shifted seed is stopped by the fitness pre-screen.
TEST_F(LoopFixture, AmbiguityCheckKeepsUniquePlaces) {
  const std::vector<common::Vec3> hall = test::hall_points_w();
  const double places[][2] = {{-14.0, 0.0}, {-8.0, -2.0}, {-3.0, 3.0}, {3.0, 0.5}, {9.0, -1.0}};
  for (const auto& pl : places) {
    Pair pr;
    pr.truth_c = body_pose(pl[0], pl[1], 0.0);
    pr.truth_q = body_pose(pl[0] + 0.2, pl[1] + 0.1, 20.0 * kDeg);
    pr.graph_c = pr.truth_c;
    pr.graph_q = body_pose(pl[0] + 0.6, pl[1] + 0.1, 20.0 * kDeg);
    const LoopResult r = run(hall, hall, pr, cfg_.loop_closure);
    EXPECT_EQ(r.outcome, LoopOutcome::kAccepted)
        << pl[0] << " " << pl[1] << ": outcome " << static_cast<int>(r.outcome) << " amb "
        << r.ambiguity_ratio;
    EXPECT_LT(r.ambiguity_ratio, cfg_.loop_closure.ambiguity_max_ratio);
    EXPECT_LE(r.n_icp_runs, 2 + 2 * cfg_.loop_closure.ambiguity_seeds_per_side);
    // Review H3 I-2: in a unique place most shifted seeds fail the fitness pre-screen (one
    // fitness pass, no ICP). Measured: 2 or 4 of the 4 seeds per place, 1 to 3 ICP runs in
    // all (the step-3 seed plus the seeds that pass).
    EXPECT_GE(r.n_prescreen_skips, 2) << pl[0] << " " << pl[1];
    EXPECT_LE(r.n_icp_runs + r.n_prescreen_skips,
              1 + 2 * cfg_.loop_closure.ambiguity_seeds_per_side);
    std::printf("[ ambiguity ] hall (%.1f, %.1f): %d ICP runs, %d seeds pre-screened out\n",
                pl[0],
                pl[1],
                r.n_icp_runs,
                r.n_prescreen_skips);
  }
}

// B-1 (review C1): the placeholder contract's constructor stays: validate-only.
TEST(LoopDetector, PlaceholderConstructorValidatesOnly) {
  LoopConfig off;  // disabled: nothing validated
  EXPECT_NO_THROW(LoopDetector{off});
  pipeline::Config cfg = test::shipped_loop_config();
  ASSERT_TRUE(cfg.loop_closure.enabled);
  LoopConfig bad = cfg.loop_closure;
  bad.search_radius_m = 0.0;
  EXPECT_THROW(LoopDetector{bad}, std::invalid_argument);
  bad = cfg.loop_closure;
  bad.min_time_gap_s = -1.0;
  EXPECT_THROW(LoopDetector{bad}, std::invalid_argument);
  // Valid and enabled: constructs, but has no matcher, so it never finds a loop.
  LoopDetector det(cfg.loop_closure);
  ScanFrame cloud(16);
  (void)cloud.push(1.0F, 0.0F, 0.0F, 0.0F, 0U, 0U);
  std::vector<KeyframeView> kfs(2);
  kfs[0].cloud_l = &cloud;
  kfs[1] = kfs[0];
  kfs[1].id = 1;
  kfs[1].t = common::Timestamp{1000 * kNs};
  EXPECT_EQ(det.detect(kfs[1], common::Span<const KeyframeView>(kfs)).outcome,
            LoopOutcome::kNoCandidate);
}

TEST(LoopDetector, DisabledIsANoOpAndValidatesNothing) {
  LoopConfig lc;  // enabled = false, every value invalid
  scan_match::ScanMatchConfig sm;
  map::MapConfig mc;
  LoopDetector det(lc, sm, mc, Pose{}, 1);
  KeyframeView q;
  EXPECT_EQ(det.detect(q, common::Span<const KeyframeView>()).outcome, LoopOutcome::kNoCandidate);
}

TEST(LoopDetector, EnabledValidatesConfig) {
  pipeline::Config cfg = test::shipped_loop_config();
  ASSERT_TRUE(cfg.loop_closure.enabled);
  LoopConfig lc = cfg.loop_closure;
  lc.search_radius_m = 0.0;
  EXPECT_THROW(LoopDetector(lc, cfg.scan_match, cfg.map, Pose{}, 10), std::invalid_argument);
  lc = cfg.loop_closure;
  lc.sc_sectors = 3;
  EXPECT_THROW(LoopDetector(lc, cfg.scan_match, cfg.map, Pose{}, 10), std::invalid_argument);
  lc = cfg.loop_closure;
  lc.min_inlier_ratio = 1.5;
  EXPECT_THROW(LoopDetector(lc, cfg.scan_match, cfg.map, Pose{}, 10), std::invalid_argument);
  lc = cfg.loop_closure;
  lc.cov_floor_rad = std::nan("");
  EXPECT_THROW(LoopDetector(lc, cfg.scan_match, cfg.map, Pose{}, 10), std::invalid_argument);
  lc = cfg.loop_closure;
  lc.correction_chi2_gate = 0.0;
  EXPECT_THROW(LoopDetector(lc, cfg.scan_match, cfg.map, Pose{}, 10), std::invalid_argument);
  lc = cfg.loop_closure;
  lc.ambiguity_seeds_per_side = -1;
  EXPECT_THROW(LoopDetector(lc, cfg.scan_match, cfg.map, Pose{}, 10), std::invalid_argument);
  lc = cfg.loop_closure;
  lc.ambiguity_max_ratio = 1.5;
  EXPECT_THROW(LoopDetector(lc, cfg.scan_match, cfg.map, Pose{}, 10), std::invalid_argument);
  lc = cfg.loop_closure;
  lc.ambiguity_chi2_margin = std::nan("");
  EXPECT_THROW(LoopDetector(lc, cfg.scan_match, cfg.map, Pose{}, 10), std::invalid_argument);
}

}  // namespace
}  // namespace mss::loop
