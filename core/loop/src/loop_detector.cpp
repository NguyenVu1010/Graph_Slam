#include "mss/loop/loop_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "mss/common/lie.hpp"
#include "mss/common/result.hpp"
#include "mss/degeneracy/config.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/scan_match/scan_matcher.hpp"

// Design: ADR-026; derivation: docs/derivations/loop_closure.md.
namespace mss::loop {

namespace {

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kPi = 3.14159265358979323846;
constexpr int kMinSectors = 4;  // fewer cannot resolve a yaw (geometry, not a tunable)

void require(bool ok, const char* key, const char* what) {
  if (!ok) {
    throw std::invalid_argument(std::string("loop_closure.") + key + " " + what);
  }
}

// Every check is written so that NaN fails it.
void validate(const LoopConfig& c) {
  require(c.search_radius_m > 0.0 && std::isfinite(c.search_radius_m),
          "search_radius_m",
          "must be > 0 and finite");
  require(c.min_time_gap_s >= 0.0 && std::isfinite(c.min_time_gap_s),
          "min_time_gap_s",
          "must be >= 0 and finite");
  require(c.max_candidates >= 1, "max_candidates", "must be >= 1");
  require(c.min_interval_keyframes >= 0, "min_interval_keyframes", "must be >= 0");
  require(c.submap_keyframes >= 0, "submap_keyframes", "must be >= 0");
  require(c.submap_max_voxels > 0, "submap_max_voxels", "must be > 0");
  require(c.sc_rings >= 1, "sc_rings", "must be >= 1");
  require(c.sc_sectors >= kMinSectors, "sc_sectors", "must be >= 4");
  require(c.sc_max_range_m > 0.0, "sc_max_range_m", "must be > 0");
  require(c.sc_height_offset_m > 0.0, "sc_height_offset_m", "must be > 0");
  require(
      c.sc_max_distance > 0.0 && c.sc_max_distance <= 1.0, "sc_max_distance", "must be in (0, 1]");
  require(c.icp_max_iterations > 0, "icp_max_iterations", "must be > 0");
  require(c.icp_max_corr_dist_m > 0.0, "icp_max_corr_dist_m", "must be > 0");
  require(c.inlier_dist_m > 0.0, "inlier_dist_m", "must be > 0");
  require(c.min_inlier_ratio > 0.0 && c.min_inlier_ratio <= 1.0,
          "min_inlier_ratio",
          "must be in (0, 1]");
  require(c.max_rms_m > 0.0, "max_rms_m", "must be > 0");
  require(c.min_hessian_eig > 0.0, "min_hessian_eig", "must be > 0");
  require(c.cov_floor_m > 0.0, "cov_floor_m", "must be > 0");
  require(c.cov_floor_rad > 0.0, "cov_floor_deg", "must be > 0");
  require(c.max_correction_m > 0.0, "max_correction_m", "must be > 0");
  require(c.max_correction_rad > 0.0, "max_correction_deg", "must be > 0");
  require(c.isam_extra_updates >= 0, "isam_extra_updates", "must be >= 0");
  require(c.correction_chi2_gate > 0.0 && std::isfinite(c.correction_chi2_gate),
          "correction_chi2_gate",
          "must be > 0 and finite");
  require(c.ambiguity_seeds_per_side >= 0, "ambiguity_seeds_per_side", "must be >= 0");
  require(c.ambiguity_seed_step_m > 0.0 && std::isfinite(c.ambiguity_seed_step_m),
          "ambiguity_seed_step_m",
          "must be > 0 and finite");
  require(c.ambiguity_min_separation_m > 0.0, "ambiguity_min_separation_m", "must be > 0");
  require(c.ambiguity_min_separation_rad > 0.0, "ambiguity_min_separation_deg", "must be > 0");
  require(c.ambiguity_max_ratio > 0.0 && c.ambiguity_max_ratio <= 1.0,
          "ambiguity_max_ratio",
          "must be in (0, 1]");
  require(c.ambiguity_chi2_margin > 0.0, "ambiguity_chi2_margin", "must be > 0");
  require(c.ambiguity_backoff_keyframes >= 0, "ambiguity_backoff_keyframes", "must be >= 0");
  require(c.ambiguity_icp_max_iterations > 0, "ambiguity_icp_max_iterations", "must be > 0");
  require(c.ambiguity_prescreen_ratio >= 0.0 && c.ambiguity_prescreen_ratio <= 1.0,
          "ambiguity_prescreen_ratio",
          "must be in [0, 1]");
}

bool pose_finite(const common::Pose& T) {
  return T.R.allFinite() && T.p.allFinite();
}

// Yaw of R about the w z axis (ZYX convention): R = Rz(yaw) * R_tilt.
double yaw_of(const common::Mat3& R) {
  return std::atan2(R(1, 0), R(0, 0));
}

common::Mat3 rot_z(double yaw) {
  common::Mat3 R;
  R << std::cos(yaw), -std::sin(yaw), 0.0,  //
      std::sin(yaw), std::cos(yaw), 0.0,    //
      0.0, 0.0, 1.0;
  return R;
}

double wrap_pi(double a) {
  return std::remainder(a, kTwoPi);
}

// Unit-aware floor (loop_closure.md section 4): C >= D^2 in the PSD sense with
// D = diag(floor_m x3, floor_rad x3). Whitens by D, raises the eigenvalues below 1 to 1,
// un-whitens.
common::PoseCovariance floor_covariance(const common::PoseCovariance& C,
                                        double floor_m,
                                        double floor_rad) {
  common::Vec6 d;
  d << floor_m, floor_m, floor_m, floor_rad, floor_rad, floor_rad;
  const common::Mat6 D = d.asDiagonal();
  const common::Mat6 D_inv = d.cwiseInverse().asDiagonal();
  const common::Mat6 W = D_inv * (0.5 * (C + C.transpose())) * D_inv;
  const Eigen::SelfAdjointEigenSolver<common::Mat6> eig(W);
  const common::Vec6 lam = eig.eigenvalues().cwiseMax(1.0);
  const common::Mat6 W_floor =
      eig.eigenvectors() * lam.asDiagonal() * eig.eigenvectors().transpose();
  const common::Mat6 out = D * W_floor * D;
  const common::Mat6 sym = 0.5 * (out + out.transpose());
  return sym;
}

}  // namespace

double scan_context_distance(common::Span<const float> q,
                             common::Span<const float> c,
                             int rings,
                             int sectors,
                             int& best_shift) {
  double best = 1.0;
  best_shift = 0;
  for (int s = 0; s < sectors; ++s) {
    double sum_cos = 0.0;
    int n = 0;
    for (int j = 0; j < sectors; ++j) {
      const int jc = (j + s) % sectors;
      double dot = 0.0;
      double nq = 0.0;
      double nc = 0.0;
      for (int r = 0; r < rings; ++r) {
        const double a = q[static_cast<std::size_t>(r * sectors + j)];
        const double b = c[static_cast<std::size_t>(r * sectors + jc)];
        dot += a * b;
        nq += a * a;
        nc += b * b;
      }
      if (nq > 0.0 && nc > 0.0) {
        sum_cos += dot / std::sqrt(nq * nc);
        ++n;
      }
    }
    const double d = n > 0 ? 1.0 - sum_cos / n : 1.0;
    if (d < best) {
      best = d;
      best_shift = s;
    }
  }
  return best;
}

struct LoopDetector::Impl {
  LoopConfig cfg;
  common::Pose T_b_l;
  common::Pose T_l_b;
  std::unique_ptr<scan_match::ScanMatcher> matcher;
  std::unique_ptr<map::VoxelMap> submap;
  std::vector<float> desc_q;
  std::vector<float> desc_c;
  // Candidates of one call: (squared distance m^2, index into keyframes). Its capacity
  // follows the largest store seen (at most map.max_keyframes), so it stops growing.
  std::vector<std::pair<double, std::size_t>> cand;
  map::NeighborBuffer nbr;

  // Scan Context of cloud_l in the gravity-levelled body frame h of T_w_b:
  // R_h_b = Rz(yaw)^T R_w_b (roll and pitch removed), origin at the body.
  void fill_descriptor(const common::ScanFrame& cloud_l,
                       const common::Pose& T_w_b,
                       std::vector<float>& out) const {
    const int rings = cfg.sc_rings;
    const int sectors = cfg.sc_sectors;
    std::fill(out.begin(), out.end(), 0.0F);
    const common::Mat3 R_tilt = rot_z(yaw_of(T_w_b.R)).transpose() * T_w_b.R;
    const common::Pose T_h_l{R_tilt * T_b_l.R, R_tilt * T_b_l.p};
    const common::Span<const float> xyz = cloud_l.xyz();
    for (std::size_t i = 0; i < cloud_l.size(); ++i) {
      const common::Vec3 p_l(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
      const common::Vec3 p_h = T_h_l * p_l;
      const double range = std::hypot(p_h.x(), p_h.y());
      if (!(range < cfg.sc_max_range_m)) {
        continue;
      }
      const int r = std::min(rings - 1, static_cast<int>(range / cfg.sc_max_range_m * rings));
      const double az = std::atan2(p_h.y(), p_h.x()) + kPi;  // [0, 2 pi]
      const int s = std::min(sectors - 1, static_cast<int>(az / kTwoPi * sectors));
      float& cell = out[static_cast<std::size_t>(r * sectors + s)];
      cell = std::max(cell, static_cast<float>(p_h.z() + cfg.sc_height_offset_m));
    }
  }

  // Inlier ratio and inlier point-to-plane RMS (m) of cloud_l at T_w_l against the submap
  // (loop_closure.md section 3), over the structure points of the query only: points whose
  // height in the levelled body frame of q (T_h_l) is above -sc_height_offset_m, the same set
  // the Scan Context sees. Ground returns fit any place and would inflate the ratio. A point
  // is an inlier when its nearest submap point is within inlier_dist_m and its knn_k
  // neighbours (at least 3) give a plane; its residual is the distance to the least-squares
  // plane through them (nearest-point distances alone are dominated by the keyframe cloud
  // spacing, map.keyframe_voxel_m).
  void fitness(const common::ScanFrame& cloud_l,
               const common::Pose& T_h_l,
               const common::Pose& T_w_l,
               double& ratio,
               double& rms) {
    const common::Span<const float> xyz = cloud_l.xyz();
    const double d2_max = cfg.inlier_dist_m * cfg.inlier_dist_m;
    double sum_r2 = 0.0;
    std::size_t n_in = 0;
    std::size_t n_struct = 0;
    for (std::size_t i = 0; i < cloud_l.size(); ++i) {
      const common::Vec3 p_l(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
      if (!((T_h_l * p_l).z() + cfg.sc_height_offset_m > 0.0)) {
        continue;
      }
      ++n_struct;
      const common::Vec3 p_w = T_w_l * p_l;
      const int k = submap->knn(p_w, 0, nbr);
      if (k < 3 || !(nbr.dist2[0] <= d2_max)) {
        continue;
      }
      common::Vec3 c = common::Vec3::Zero();
      for (int n = 0; n < k; ++n) {
        c += nbr.p_w[static_cast<std::size_t>(n)];
      }
      c /= static_cast<double>(k);
      common::Mat3 S = common::Mat3::Zero();
      for (int n = 0; n < k; ++n) {
        const common::Vec3 d = nbr.p_w[static_cast<std::size_t>(n)] - c;
        S += d * d.transpose();
      }
      const Eigen::SelfAdjointEigenSolver<common::Mat3> eig(S);
      const common::Vec3 normal = eig.eigenvectors().col(0);
      const double r = normal.dot(p_w - c);
      sum_r2 += r * r;
      ++n_in;
    }
    ratio = n_struct > 0 ? static_cast<double>(n_in) / static_cast<double>(n_struct) : 0.0;
    rms = n_in > 0 ? std::sqrt(sum_r2 / static_cast<double>(n_in))
                   : std::numeric_limits<double>::infinity();
  }

  // One converged ICP fit of the query within the fixed correction gates (section 9: it may
  // still fail the chi-square gate, then plausible = false).
  struct Fit {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    common::MatchResult mr;
    common::Pose T_w_bq;                                    // fitted body pose of q
    common::PoseCovariance cov = common::Mat6::Identity();  // floored, b_q (section 4)
    double ratio = 0.0;
    double rms = 0.0;
    double chi2 = std::numeric_limits<double>::quiet_NaN();  // NaN without a source
    bool plausible = true;  // chi2 <= correction_chi2_gate (true without a source)
  };
  enum class GateOutcome : std::uint8_t { kOk, kFailed, kTooFar, kNoMarginal };

  // Correction gates of one candidate (loop_closure.md sections 3 and 9). The graph's relative
  // covariance is asked for at most once per candidate, when a fit first needs it.
  struct Gate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Gate(const Impl& impl,
         const KeyframeView& cand,
         const KeyframeView& query,
         const RelativeCovarianceSource* source)
        : m(impl),
          c(cand),
          q(query),
          rel(source),
          T_c_q_graph(cand.T_w_b.inverse() * query.T_w_b),
          yaw_q(yaw_of(query.T_w_b.R)) {}

    // The fixed gates (kTooFar), then e^T S^-1 e: fills f.cov, f.chi2 and f.plausible. A fit
    // that fails only the chi-square gate is kOk with plausible = false: it cannot be the loop
    // but it can still compete with one (ambiguity check).
    GateOutcome check(Fit& f) {
      const LoopConfig& cf = m.cfg;
      if (!((f.T_w_bq.p - q.T_w_b.p).norm() <= cf.max_correction_m) ||
          !(std::abs(wrap_pi(yaw_of(f.T_w_bq.R) - yaw_q)) <= cf.max_correction_rad)) {
        return GateOutcome::kTooFar;
      }
      const common::PoseCovariance cov_b = common::transform_covariance(f.mr.cov, m.T_b_l);
      f.cov = floor_covariance(cov_b, cf.cov_floor_m, cf.cov_floor_rad);
      f.chi2 = std::numeric_limits<double>::quiet_NaN();
      f.plausible = true;
      if (rel == nullptr) {
        return GateOutcome::kOk;
      }
      if (!asked) {
        asked = true;
        const common::Result<common::PoseCovariance> r = rel->relative_cov(c.id, q.id);
        have_rel = r.is_ok() && r.value().allFinite();
        if (have_rel) {
          sigma_rel = r.value();
        }
      }
      if (!have_rel) {
        return GateOutcome::kNoMarginal;
      }
      // e: the fitted relative pose against the graph's, right perturbation of T_c_q (b_q).
      const common::Pose T_c_q_fit = c.T_w_b.inverse() * f.T_w_bq;
      const common::Vec6 e = common::boxminus(T_c_q_fit, T_c_q_graph);
      const common::Mat6 S = 0.5 * (sigma_rel + sigma_rel.transpose()) + f.cov;
      const Eigen::LDLT<common::Mat6> ldlt(S);
      f.chi2 = std::numeric_limits<double>::infinity();
      if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
        const common::Vec6 S_inv_e = ldlt.solve(e);
        f.chi2 = e.dot(S_inv_e);
      }
      f.plausible = f.chi2 <= cf.correction_chi2_gate;
      return GateOutcome::kOk;
    }

    // chi2 of a fit at body pose T_w_bq with the covariance `cov` (no ICP): the value the
    // chi-square gate would give it. +inf when S is not positive definite. Precondition: the
    // marginal is available (have_rel).
    double predicted_chi2(const common::Pose& T_w_bq, const common::PoseCovariance& cov) const {
      const common::Vec6 e = common::boxminus(c.T_w_b.inverse() * T_w_bq, T_c_q_graph);
      const common::Mat6 S = 0.5 * (sigma_rel + sigma_rel.transpose()) + cov;
      const Eigen::LDLT<common::Mat6> ldlt(S);
      if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        return std::numeric_limits<double>::infinity();
      }
      const common::Vec6 S_inv_e = ldlt.solve(e);
      return e.dot(S_inv_e);
    }

    const Impl& m;
    const KeyframeView& c;
    const KeyframeView& q;
    const RelativeCovarianceSource* rel;
    const common::Pose T_c_q_graph;
    const double yaw_q;
    bool asked = false;
    bool have_rel = false;
    common::PoseCovariance sigma_rel = common::PoseCovariance::Zero();
  };

  // ICP (with `mt`) from the body seed `seed_w_b`, then the correction gates and the fitness.
  // `need_converged` false: a fit that ran out of iterations is still judged by its fitness
  // (ambiguity seeds, ambiguity_icp_max_iterations: a competitor only has to reach a basin).
  GateOutcome fit_from(scan_match::ScanMatcher& mt,
                       bool need_converged,
                       const common::ScanFrame& cloud_l,
                       const common::Pose& T_h_l,
                       const common::Pose& seed_w_b,
                       Gate& gate,
                       Fit& f,
                       LoopResult& res) {
    ++res.n_icp_runs;
    const common::Result<common::MatchResult> mr = mt.align(cloud_l, *submap, seed_w_b * T_b_l);
    if (!mr.is_ok() || (need_converged && !mr.value().converged)) {
      return GateOutcome::kFailed;
    }
    f.mr = mr.value();
    f.T_w_bq = f.mr.T_w_l * T_l_b;
    const GateOutcome g = gate.check(f);
    if (g != GateOutcome::kOk) {
      return g;
    }
    fitness(cloud_l, T_h_l, f.mr.T_w_l, f.ratio, f.rms);
    return GateOutcome::kOk;
  }

  // Inlier ratio of `other` over that of `best` when `other` competes with it, else 0. It
  // competes when it is separated from `best` by more than ambiguity_min_separation_m or _rad,
  // its inlier RMS is within max_rms_m, and the graph does not rule it out. With a
  // RelativeCovarianceSource the graph rules it out only when both hold: `other` fails the
  // chi-square gate (other.chi2 > correction_chi2_gate) and is at least
  // ambiguity_chi2_margin less likely than `best` in chi-square (likelihood ratio
  // exp(margin / 2)). So a fit the graph finds plausible always competes: while the true drift
  // is within the gate, a false fit cannot win against the true one (section 10).
  double competitor_ratio(const Fit& best, const Fit& other) const {
    const common::Pose d = best.T_w_bq.inverse() * other.T_w_bq;
    const bool separated = d.p.norm() > cfg.ambiguity_min_separation_m ||
                           common::so3_log(d.R).norm() > cfg.ambiguity_min_separation_rad;
    if (!separated || !(other.rms <= cfg.max_rms_m) || !(best.ratio > 0.0)) {
      return 0.0;
    }
    if (std::isfinite(best.chi2) && !(other.chi2 <= cfg.correction_chi2_gate) &&
        !(other.chi2 - best.chi2 < cfg.ambiguity_chi2_margin)) {
      return 0.0;  // the graph rules `other` out
    }
    return other.ratio / best.ratio;
  }

  // Ambiguity search (loop_closure.md section 10): ICP from the best fit shifted by
  // +-k * ambiguity_seed_step_m, k = 1..ambiguity_seeds_per_side, nearest first, along u: the
  // horizontal direction (in w) with the least translation information in the best fit's
  // Hessian, rotation marginalised out (Schur complement). Seeds farther than max_correction_m
  // from the graph pose of q are skipped, and so are seeds the graph rules out (above).
  // Returns the largest competitor ratio; stops at the first one >= ambiguity_max_ratio.
  double ambiguity_search(const common::ScanFrame& cloud_l,
                          const common::Pose& T_h_l,
                          const KeyframeView& query,
                          const Fit& best,
                          Gate& gate,
                          LoopResult& res) {
    if (cfg.ambiguity_seeds_per_side <= 0) {
      return 0.0;
    }
    const common::Mat6 H = 0.5 * (best.mr.H + best.mr.H.transpose());
    const common::Mat3 H_tt = H.topLeftCorner<3, 3>();
    const common::Mat3 H_tr = H.topRightCorner<3, 3>();
    const common::Mat3 H_rr = H.bottomRightCorner<3, 3>();
    const Eigen::LDLT<common::Mat3> ldlt_rr(H_rr);
    const common::Mat3 H_rt_solved = ldlt_rr.solve(H_tr.transpose());
    const common::Mat3 H_t_l = H_tt - H_tr * H_rt_solved;  // translation information, in l
    const common::Mat3& R_w_l = best.mr.T_w_l.R;
    const common::Mat3 H_t_w = R_w_l * H_t_l * R_w_l.transpose();
    const Eigen::Matrix2d H_xy_raw = H_t_w.topLeftCorner<2, 2>();
    const Eigen::Matrix2d H_xy = 0.5 * (H_xy_raw + H_xy_raw.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(H_xy);
    const Eigen::Vector2d u2 = eig.eigenvectors().col(0);  // smallest eigenvalue first
    const common::Vec3 u(u2.x(), u2.y(), 0.0);
    if (!u.allFinite()) {
      return 0.0;
    }
    double amb = 0.0;
    for (int k = 1; k <= cfg.ambiguity_seeds_per_side; ++k) {
      for (const double sign : {1.0, -1.0}) {
        const common::Vec3 p = best.T_w_bq.p + (sign * k * cfg.ambiguity_seed_step_m) * u;
        if (!((p - query.T_w_b.p).norm() <= cfg.max_correction_m)) {
          continue;
        }
        // With the graph's covariance: a seed whose whole reach (its offset minus half a step,
        // towards the best fit) the graph already rules out (chi2 above the gate and
        // ambiguity_chi2_margin above the best fit's) cannot give a competitor: skipped.
        if (gate.have_rel && std::isfinite(best.chi2)) {
          const common::Vec3 p_in =
              best.T_w_bq.p + (sign * (k - 0.5) * cfg.ambiguity_seed_step_m) * u;
          const double chi2_in = gate.predicted_chi2(common::Pose{best.T_w_bq.R, p_in}, best.cov);
          if (chi2_in > cfg.correction_chi2_gate &&
              chi2_in - best.chi2 >= cfg.ambiguity_chi2_margin) {
            continue;
          }
        }
        // Cheap pre-screen (review H3, I-2): one fitness pass at the shifted pose, no ICP. A seed
        // whose raw inlier ratio is below ambiguity_prescreen_ratio x the best fit's is not near
        // a copy that fits as well: no ICP for it.
        const common::Pose seed{best.T_w_bq.R, p};
        if (cfg.ambiguity_prescreen_ratio > 0.0) {
          double pre_ratio = 0.0;
          double pre_rms = 0.0;
          fitness(cloud_l, T_h_l, seed * T_b_l, pre_ratio, pre_rms);
          if (!(pre_ratio >= cfg.ambiguity_prescreen_ratio * best.ratio)) {
            ++res.n_prescreen_skips;
            continue;
          }
        }
        if (fit_from(*amb_matcher, false, cloud_l, T_h_l, seed, gate, amb_fit, res) !=
            GateOutcome::kOk) {
          continue;
        }
        amb = std::max(amb, competitor_ratio(best, amb_fit));
        if (amb >= cfg.ambiguity_max_ratio) {
          return amb;
        }
      }
    }
    return amb;
  }

  // ICP of the ambiguity seeds: max_iterations = ambiguity_icp_max_iterations.
  std::unique_ptr<scan_match::ScanMatcher> amb_matcher;
  std::array<Fit, 2> fits;  // the step-3 seeds' fits of one candidate
  Fit amb_fit;              // scratch of ambiguity_search
};

LoopDetector::LoopDetector(const LoopConfig& cfg,
                           const scan_match::ScanMatchConfig& sm,
                           const map::MapConfig& map_cfg,
                           const common::Pose& T_b_l,
                           std::size_t max_points)
    : impl_(std::make_unique<Impl>()) {
  Impl& m = *impl_;
  m.cfg = cfg;
  m.T_b_l = T_b_l;
  m.T_l_b = T_b_l.inverse();
  if (!cfg.enabled) {
    return;
  }
  validate(cfg);
  scan_match::ScanMatchConfig icp = sm;
  icp.max_iterations = cfg.icp_max_iterations;
  icp.max_corr_dist_m = cfg.icp_max_corr_dist_m;
  degeneracy::DegeneracyConfig no_deg;  // the acceptance gate reads H; no projected update
  no_deg.enabled = false;
  m.matcher = std::make_unique<scan_match::ScanMatcher>(icp, no_deg, max_points);
  scan_match::ScanMatchConfig amb_icp = icp;
  amb_icp.max_iterations = cfg.ambiguity_icp_max_iterations;
  m.amb_matcher = std::make_unique<scan_match::ScanMatcher>(amb_icp, no_deg, max_points);
  map::MapConfig sub = map_cfg;
  sub.max_voxels = cfg.submap_max_voxels;
  m.submap = std::make_unique<map::VoxelMap>(sub);
  const std::size_t cells =
      static_cast<std::size_t>(cfg.sc_rings) * static_cast<std::size_t>(cfg.sc_sectors);
  m.desc_q.assign(cells, 0.0F);
  m.desc_c.assign(cells, 0.0F);
}

LoopDetector::LoopDetector(const LoopConfig& cfg) : impl_(std::make_unique<Impl>()) {
  impl_->cfg = cfg;
  if (cfg.enabled) {
    validate(cfg);
  }
}

LoopDetector::~LoopDetector() = default;
LoopDetector::LoopDetector(LoopDetector&&) noexcept = default;
LoopDetector& LoopDetector::operator=(LoopDetector&&) noexcept = default;

void LoopDetector::descriptor(const common::ScanFrame& cloud_l,
                              const common::Pose& T_w_b,
                              std::vector<float>& out) const {
  out.assign(static_cast<std::size_t>(impl_->cfg.sc_rings) *
                 static_cast<std::size_t>(impl_->cfg.sc_sectors),
             0.0F);
  impl_->fill_descriptor(cloud_l, T_w_b, out);
}

LoopResult LoopDetector::detect(const KeyframeView& query,
                                common::Span<const KeyframeView> keyframes) {
  return detect(query, keyframes, nullptr);
}

LoopResult LoopDetector::detect(const KeyframeView& query,
                                common::Span<const KeyframeView> keyframes,
                                const RelativeCovarianceSource* rel) {
  Impl& m = *impl_;
  LoopResult res;
  if (!m.cfg.enabled || m.matcher == nullptr || query.cloud_l == nullptr ||
      query.cloud_l->size() == 0 || !pose_finite(query.T_w_b)) {
    return res;
  }
  const LoopConfig& c = m.cfg;
  const std::int64_t gap_ns = static_cast<std::int64_t>(std::llround(c.min_time_gap_s * 1e9));
  const auto old_enough = [&](const KeyframeView& k) { return k.t.ns <= query.t.ns - gap_ns; };

  // 1. Candidates by distance on the optimised poses.
  m.cand.clear();
  if (m.cand.capacity() < keyframes.size()) {
    m.cand.reserve(keyframes.size());
  }
  const double r2 = c.search_radius_m * c.search_radius_m;
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    const KeyframeView& k = keyframes[i];
    if (k.id == query.id || !old_enough(k) || k.cloud_l == nullptr || k.cloud_l->size() == 0 ||
        !pose_finite(k.T_w_b)) {
      continue;
    }
    const double d2 = (k.T_w_b.p - query.T_w_b.p).squaredNorm();
    if (d2 <= r2) {
      m.cand.emplace_back(d2, i);
    }
  }
  res.n_candidates = static_cast<int>(m.cand.size());
  if (m.cand.empty()) {
    return res;
  }
  std::sort(m.cand.begin(), m.cand.end());  // nearest first; ties by store order
  const std::size_t n_try = std::min(m.cand.size(), static_cast<std::size_t>(c.max_candidates));

  m.fill_descriptor(*query.cloud_l, query.T_w_b, m.desc_q);
  const double sector_rad = kTwoPi / c.sc_sectors;
  const double yaw_q = yaw_of(query.T_w_b.R);
  const common::Mat3 R_tilt_q = rot_z(yaw_q).transpose() * query.T_w_b.R;
  const common::Pose T_h_l_q{R_tilt_q * m.T_b_l.R, R_tilt_q * m.T_b_l.p};
  bool verified_any = false;
  res.outcome = LoopOutcome::kDescriptor;
  for (std::size_t n = 0; n < n_try; ++n) {
    const std::size_t ci = m.cand[n].second;
    const KeyframeView& cand = keyframes[ci];
    // 2. Scan Context gate and relative yaw (query - candidate).
    m.fill_descriptor(*cand.cloud_l, cand.T_w_b, m.desc_c);
    int shift = 0;
    const double sc_d = scan_context_distance(common::Span<const float>(m.desc_q),
                                              common::Span<const float>(m.desc_c),
                                              c.sc_rings,
                                              c.sc_sectors,
                                              shift);
    const double sc_yaw = wrap_pi(shift * sector_rad);
    if (!verified_any) {
      res.candidate = cand.id;
      res.candidate_t = cand.t;
      res.sc_distance = sc_d;
      res.sc_yaw_rad = sc_yaw;
    }
    if (sc_d > c.sc_max_distance) {
      continue;
    }
    // 3. Submap: the candidate and its old-enough store neighbours at their optimised poses.
    m.submap->clear();
    const std::size_t span = static_cast<std::size_t>(c.submap_keyframes);
    const std::size_t lo = ci >= span ? ci - span : 0;
    const std::size_t hi = std::min(keyframes.size() - 1, ci + span);
    for (std::size_t k = lo; k <= hi; ++k) {
      const KeyframeView& kv = keyframes[k];
      if (kv.cloud_l == nullptr || !old_enough(kv) || !pose_finite(kv.T_w_b)) {
        continue;
      }
      m.submap->insert(*kv.cloud_l, kv.T_w_b * m.T_b_l);
    }
    ++res.n_verified;
    // ADR-028: the graph's covariance is used only with graph_covariance_gate (default off).
    Impl::Gate gate(m, cand, query, c.graph_covariance_gate ? rel : nullptr);
    // Seeds: the graph pose of q, and q with the Scan Context yaw when that differs from
    // the graph yaw by more than one sector.
    const double yaw_seed = wrap_pi(yaw_of(cand.T_w_b.R) + sc_yaw);
    std::array<common::Pose, 2> seeds;
    seeds[0] = query.T_w_b;
    std::size_t n_seeds = 1;
    if (std::abs(wrap_pi(yaw_seed - yaw_q)) > sector_rad) {
      seeds[1] = common::Pose{rot_z(yaw_seed) * R_tilt_q, query.T_w_b.p};
      n_seeds = 2;
    }
    // Every fit within the fixed gates is kept (a competitor); the best is the plausible one
    // (chi-square gate) with the highest inlier ratio.
    std::size_t n_fits = 0;
    std::size_t best = 0;
    bool have = false;
    bool too_far = false;
    bool no_marginal = false;
    for (std::size_t s = 0; s < n_seeds; ++s) {
      Impl::Fit& f = m.fits[n_fits];
      const Impl::GateOutcome g =
          m.fit_from(*m.matcher, true, *query.cloud_l, T_h_l_q, seeds[s], gate, f, res);
      if (g == Impl::GateOutcome::kTooFar) {
        too_far = true;
      } else if (g == Impl::GateOutcome::kNoMarginal) {
        no_marginal = true;
      } else if (g == Impl::GateOutcome::kOk) {
        if (!f.plausible) {
          too_far = true;
        } else if (!have || f.ratio > m.fits[best].ratio) {
          best = n_fits;
          have = true;
        }
        ++n_fits;
      }
    }
    // 4. Acceptance gates, then the ambiguity check.
    LoopOutcome out = LoopOutcome::kIcpFailed;
    if (no_marginal) {
      out = LoopOutcome::kNoMarginal;
    } else if (too_far) {
      out = LoopOutcome::kCorrection;
    }
    double min_eig = 0.0;
    double amb = 0.0;
    const Impl::Fit& b = m.fits[best];
    if (have) {
      const common::Mat6 H_sym = 0.5 * (b.mr.H + b.mr.H.transpose());
      const Eigen::SelfAdjointEigenSolver<common::Mat6> eig(H_sym, Eigen::EigenvaluesOnly);
      min_eig = eig.eigenvalues()(0);
      if (b.ratio < c.min_inlier_ratio) {
        out = LoopOutcome::kInliers;
      } else if (!(b.rms <= c.max_rms_m)) {
        out = LoopOutcome::kRms;
      } else if (!(min_eig >= c.min_hessian_eig)) {
        out = LoopOutcome::kDegenerate;
      } else {
        // The other step-3 fit first, then the fits from the shifted seeds.
        for (std::size_t s = 0; s < n_fits; ++s) {
          if (s != best) {
            amb = std::max(amb, m.competitor_ratio(b, m.fits[s]));
          }
        }
        if (amb < c.ambiguity_max_ratio) {
          amb = std::max(amb, m.ambiguity_search(*query.cloud_l, T_h_l_q, query, b, gate, res));
        }
        out = amb >= c.ambiguity_max_ratio ? LoopOutcome::kAmbiguous : LoopOutcome::kAccepted;
      }
    }
    verified_any = true;
    res.candidate = cand.id;
    res.candidate_t = cand.t;
    res.sc_distance = sc_d;
    res.sc_yaw_rad = sc_yaw;
    res.outcome = out;
    res.inlier_ratio = have ? b.ratio : 0.0;
    res.rms_m = have ? b.rms : 0.0;
    res.min_eig = min_eig;
    res.correction_chi2 = have         ? b.chi2
                          : n_fits > 0 ? m.fits[0].chi2
                                       : std::numeric_limits<double>::quiet_NaN();
    res.ambiguity_ratio = amb;
    if (out != LoopOutcome::kAccepted) {
      continue;
    }
    // 5. Measurement T_bc_bq and its covariance in b_q, floored (computed by the gate).
    res.constraint.i = cand.id;
    res.constraint.j = query.id;
    res.constraint.T_bi_bj = cand.T_w_b.inverse() * b.T_w_bq;
    res.constraint.cov = b.cov;
    if (!pose_finite(res.constraint.T_bi_bj) || !res.constraint.cov.allFinite()) {
      res.outcome = LoopOutcome::kIcpFailed;
      continue;
    }
    return res;
  }
  return res;
}

}  // namespace mss::loop
