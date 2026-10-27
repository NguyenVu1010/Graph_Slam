#include "mss/scan_match/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

#include <Eigen/Cholesky>

#include "mss/common/lie.hpp"
#include "mss/degeneracy/degeneracy.hpp"

#include "plane_fit.hpp"

// Derivation: docs/derivations/scan_match.md. Tangent [rho, phi] of the right perturbation
// of T_w_l (frame l) for H, g, xi and cov. Hot path (align, build_system): fixed-size Eigen,
// no allocation, logging or exceptions.
namespace mss::scan_match {

namespace {

constexpr int kPoseDof = 6;  // SE(3) degrees of freedom (geometry, not a tunable)

bool pose_finite(const common::Pose& pose) {
  return pose.R.allFinite() && pose.p.allFinite();
}

}  // namespace

ScanMatcher::ScanMatcher(const ScanMatchConfig& cfg,
                         const degeneracy::DegeneracyConfig& deg_cfg,
                         std::size_t max_points)
    : cfg_(cfg), deg_cfg_(deg_cfg), max_points_(max_points) {
  // Written as !(x > 0) so that NaN is rejected too.
  if (!(cfg.max_iterations > 0)) {
    throw std::invalid_argument("scan_match.max_iterations must be > 0");
  }
  if (!(cfg.rot_tol_rad > 0.0)) {
    throw std::invalid_argument("scan_match.rot_tol_rad must be > 0");
  }
  if (!(cfg.trans_tol_m > 0.0)) {
    throw std::invalid_argument("scan_match.trans_tol_m must be > 0");
  }
  if (!(cfg.max_corr_dist_m > 0.0)) {
    throw std::invalid_argument("scan_match.max_corr_dist_m must be > 0");
  }
  if (!(cfg.plane_rms_max_m > 0.0)) {
    throw std::invalid_argument("scan_match.plane_rms_max_m must be > 0");
  }
  if (!(cfg.plane_min_eig_ratio >= 0.0 && cfg.plane_min_eig_ratio < 1.0)) {
    throw std::invalid_argument("scan_match.plane_min_eig_ratio must be in [0, 1)");
  }
  if (!(cfg.min_mid_eig_ratio >= 0.0 && cfg.min_mid_eig_ratio < 1.0)) {
    throw std::invalid_argument("scan_match.min_mid_eig_ratio must be in [0, 1)");
  }
  if (!(cfg.robust_huber_delta_m > 0.0)) {
    throw std::invalid_argument("scan_match.robust_huber_delta_m must be > 0");
  }
  if (cfg.min_inliers < kPoseDof) {
    throw std::invalid_argument("scan_match.min_inliers must be >= 6 (pose DoF)");
  }
  if (!(cfg.cov_scale > 0.0)) {
    throw std::invalid_argument("scan_match.cov_scale must be > 0");
  }
  if (!(cfg.min_residual_sigma_m > 0.0)) {
    throw std::invalid_argument("scan_match.min_residual_sigma_m must be > 0");
  }
  if (max_points == 0) {
    throw std::invalid_argument("scan_match: max_points must be > 0");
  }
  p_l_.resize(max_points);
}

int ScanMatcher::build_system(const map::VoxelMap& map,
                              std::size_t n,
                              const common::Pose& T_w_l,
                              common::Mat6& hessian,
                              common::Vec6& gradient,
                              double& sum_w_r2_m2,
                              double& sum_w) {
  hessian.setZero();
  gradient.setZero();
  sum_w_r2_m2 = 0.0;
  sum_w = 0.0;
  const double max_corr_dist2_m2 = cfg_.max_corr_dist_m * cfg_.max_corr_dist_m;
  const double delta_m = cfg_.robust_huber_delta_m;
  int inliers = 0;
  common::Vec6 J_r_xi = common::Vec6::Zero();
  for (std::size_t j = 0; j < n; ++j) {
    const common::Vec3& p_l = p_l_[j];
    const common::Vec3 p_w = T_w_l * p_l;
    const int k = map.knn(p_w, 0, nbr_);
    if (k < detail::kMinPlanePoints || static_cast<double>(nbr_.dist2[0]) > max_corr_dist2_m2) {
      continue;
    }
    const std::optional<detail::PlaneFit> fit = detail::fit_plane(nbr_);
    if (!fit.has_value() || !(fit->rms_m <= cfg_.plane_rms_max_m) || !fit->n_w.allFinite() ||
        !detail::passes_planarity(*fit, cfg_.plane_min_eig_ratio) ||
        !detail::passes_collinearity(*fit, cfg_.min_mid_eig_ratio)) {
      continue;
    }
    const double r = point_to_plane_residual(p_l, fit->q_w, fit->n_w, T_w_l, &J_r_xi);
    const double abs_r = std::abs(r);
    const double w = abs_r <= delta_m ? 1.0 : delta_m / abs_r;
    hessian.noalias() += w * (J_r_xi * J_r_xi.transpose());
    gradient.noalias() += (w * r) * J_r_xi;
    sum_w_r2_m2 += w * r * r;
    sum_w += w;
    ++inliers;
  }
  return inliers;
}

common::Result<common::MatchResult> ScanMatcher::align(const common::ScanFrame& scan_l,
                                                       const map::VoxelMap& map,
                                                       const common::Pose& T_w_l_init) {
  using R = common::Result<common::MatchResult>;
  if (scan_l.size() > max_points_) {
    return R::error("scan larger than reserved max_points");
  }
  if (!pose_finite(T_w_l_init)) {
    return R::error("non-finite initial pose");
  }
  // Promote the finite scan points to double once per align; non-finite ones are skipped.
  const common::Span<const float> xyz = scan_l.xyz();
  std::size_t n = 0;
  for (std::size_t i = 0; i < scan_l.size(); ++i) {
    const float x = xyz[3 * i];
    const float y = xyz[3 * i + 1];
    const float z = xyz[3 * i + 2];
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      p_l_[n++] =
          common::Vec3(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
    }
  }

  common::MatchResult result;
  common::Pose T_w_l = T_w_l_init;
  common::Mat6 hessian = common::Mat6::Zero();
  common::Vec6 g = common::Vec6::Zero();
  Eigen::LLT<common::Mat6> llt;
  double sum_w_r2_m2 = 0.0;
  double sum_w = 0.0;
  common::DegeneracyReport report;  // last iteration's analysis (enabled only)
  for (int it = 1; it <= cfg_.max_iterations; ++it) {
    const int inliers = build_system(map, n, T_w_l, hessian, g, sum_w_r2_m2, sum_w);
    if (inliers < cfg_.min_inliers) {
      return R::error("too few inliers");
    }
    common::Vec6 xi = common::Vec6::Zero();
    if (deg_cfg_.enabled) {
      // Raise flagged eigenvalues to eig_threshold, solve, then project (derivation
      // section 5). With nothing flagged hessian_reg == hessian bit for bit.
      report = degeneracy::analyze(hessian, deg_cfg_);
      common::Mat6 hessian_reg = hessian;
      for (int k = 0; k < kPoseDof; ++k) {
        if (report.degenerate[static_cast<std::size_t>(k)]) {
          const common::Vec6 v_k = report.eigvecs.col(k);
          hessian_reg.noalias() +=
              (deg_cfg_.eig_threshold - report.eigvals(k)) * (v_k * v_k.transpose());
        }
      }
      llt.compute(hessian_reg);
      if (llt.info() != Eigen::Success) {
        return R::error("singular Hessian");
      }
      const common::Vec6 xi_full = llt.solve(-g);
      xi = degeneracy::project_update(xi_full, report);
    } else {
      llt.compute(hessian);
      if (llt.info() != Eigen::Success) {
        return R::error("singular Hessian");
      }
      xi = llt.solve(-g);
    }
    if (!xi.allFinite()) {
      return R::error("singular Hessian");
    }
    T_w_l = common::boxplus(T_w_l, xi);
    result.H = hessian;
    result.inliers = inliers;
    result.iterations = it;
    if (xi.head<3>().norm() < cfg_.trans_tol_m && xi.tail<3>().norm() < cfg_.rot_tol_rad) {
      result.converged = true;
      break;
    }
  }
  // cov = cov_scale * max(sigma_r^2, min_residual_sigma_m^2) * A^-1 (A = H or H_reg) from the
  // last factorisation, then inflated along flagged directions (derivation section 6).
  const double sigma_floor_m2 = cfg_.min_residual_sigma_m * cfg_.min_residual_sigma_m;
  const double sigma_r2_m2 = std::max(sum_w_r2_m2 / sum_w, sigma_floor_m2);
  const common::Mat6 a_inv = llt.solve(common::Mat6::Identity());
  const common::Mat6 cov = (cfg_.cov_scale * sigma_r2_m2) * a_inv;
  const common::Mat6 cov_sym = 0.5 * (cov + cov.transpose());
  result.cov = deg_cfg_.enabled ? degeneracy::relax_covariance(cov_sym, report, deg_cfg_) : cov_sym;
  result.T_w_l = T_w_l;
  if (!pose_finite(T_w_l) || !result.cov.allFinite()) {
    return R::error("singular Hessian");
  }
  return R::ok(result);
}

}  // namespace mss::scan_match
