#include "mss/loop/loop_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

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
  map::MapConfig sub = map_cfg;
  sub.max_voxels = cfg.submap_max_voxels;
  m.submap = std::make_unique<map::VoxelMap>(sub);
  const std::size_t cells =
      static_cast<std::size_t>(cfg.sc_rings) * static_cast<std::size_t>(cfg.sc_sectors);
  m.desc_q.assign(cells, 0.0F);
  m.desc_c.assign(cells, 0.0F);
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
  Impl& m = *impl_;
  LoopResult res;
  if (!m.cfg.enabled || query.cloud_l == nullptr || query.cloud_l->size() == 0 ||
      !pose_finite(query.T_w_b)) {
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
    bool have = false;
    bool too_far = false;
    common::MatchResult best;
    double best_ratio = -1.0;
    double best_rms = 0.0;
    for (std::size_t s = 0; s < n_seeds; ++s) {
      const common::Result<common::MatchResult> mr =
          m.matcher->align(*query.cloud_l, *m.submap, seeds[s] * m.T_b_l);
      if (!mr.is_ok() || !mr.value().converged) {
        continue;
      }
      // Correction gate: the ICP pose of q against its graph pose (the drift since c).
      const common::Pose T_w_bq_icp = mr.value().T_w_l * m.T_l_b;
      if (!((T_w_bq_icp.p - query.T_w_b.p).norm() <= c.max_correction_m) ||
          !(std::abs(wrap_pi(yaw_of(T_w_bq_icp.R) - yaw_q)) <= c.max_correction_rad)) {
        too_far = true;
        continue;
      }
      double ratio = 0.0;
      double rms = 0.0;
      m.fitness(*query.cloud_l, T_h_l_q, mr.value().T_w_l, ratio, rms);
      if (ratio > best_ratio) {
        best_ratio = ratio;
        best_rms = rms;
        best = mr.value();
        have = true;
      }
    }
    // 4. Acceptance gates.
    LoopOutcome out = too_far ? LoopOutcome::kCorrection : LoopOutcome::kIcpFailed;
    double min_eig = 0.0;
    if (have) {
      const common::Mat6 H_sym = 0.5 * (best.H + best.H.transpose());
      const Eigen::SelfAdjointEigenSolver<common::Mat6> eig(H_sym, Eigen::EigenvaluesOnly);
      min_eig = eig.eigenvalues()(0);
      if (best_ratio < c.min_inlier_ratio) {
        out = LoopOutcome::kInliers;
      } else if (!(best_rms <= c.max_rms_m)) {
        out = LoopOutcome::kRms;
      } else if (!(min_eig >= c.min_hessian_eig)) {
        out = LoopOutcome::kDegenerate;
      } else {
        out = LoopOutcome::kAccepted;
      }
    }
    verified_any = true;
    res.candidate = cand.id;
    res.candidate_t = cand.t;
    res.sc_distance = sc_d;
    res.sc_yaw_rad = sc_yaw;
    res.outcome = out;
    res.inlier_ratio = have ? best_ratio : 0.0;
    res.rms_m = have ? best_rms : 0.0;
    res.min_eig = min_eig;
    if (out != LoopOutcome::kAccepted) {
      continue;
    }
    // 5. Measurement T_bc_bq and its covariance in b_q, floored.
    const common::Pose T_w_bq = best.T_w_l * m.T_l_b;
    res.constraint.i = cand.id;
    res.constraint.j = query.id;
    res.constraint.T_bi_bj = cand.T_w_b.inverse() * T_w_bq;
    const common::PoseCovariance cov_b = common::transform_covariance(best.cov, m.T_b_l);
    res.constraint.cov = floor_covariance(cov_b, c.cov_floor_m, c.cov_floor_rad);
    if (!pose_finite(res.constraint.T_bi_bj) || !res.constraint.cov.allFinite()) {
      res.outcome = LoopOutcome::kIcpFailed;
      continue;
    }
    return res;
  }
  return res;
}

}  // namespace mss::loop
