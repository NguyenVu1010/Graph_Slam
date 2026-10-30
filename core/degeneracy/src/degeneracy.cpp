#include "mss/degeneracy/degeneracy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <Eigen/Eigenvalues>

// Derivation: docs/derivations/degeneracy.md. Tangent [rho, phi] of the pose H was built
// for (right perturbation); no frame conversion here. Hot path: fixed-size Eigen only.
namespace mss::degeneracy {
namespace {

constexpr int kDim = 6;
// Relative size below which an eigenvalue of H (against the largest) or a component of a
// direction (against its squared norm) is numerical round-off: far below any scan's
// information ratio (corridor axis against walls: about 1e-4) and far above double precision
// round-off of a 6x6 eigen-solve (about 1e-15).
constexpr double kNullRelative = 1e-9;

// True if every entry of the lower triangle (i >= j) is finite. The upper triangle is
// never read (header precondition: H symmetric, lower triangle used).
bool lower_triangle_finite(const common::Mat6& hessian) {
  for (int j = 0; j < kDim; ++j) {
    for (int i = j; i < kDim; ++i) {
      if (!std::isfinite(hessian(i, j))) {
        return false;
      }
    }
  }
  return true;
}

bool any_flagged(const common::DegeneracyReport& report) {
  for (const bool flag : report.degenerate) {
    if (flag) {
      return true;
    }
  }
  return false;
}

// max(var, relax_factor * c / eig_threshold): the pre-PR-4b relaxed variance as a floor.
// No floor when eig_threshold <= 0.
double floored_variance(double var, const DegeneracyConfig& cfg, double c) {
  return cfg.eig_threshold > 0.0 ? std::max(var, cfg.relax_factor * c / cfg.eig_threshold) : var;
}

}  // namespace

common::DegeneracyReport analyze(const common::Mat6& hessian, const DegeneracyConfig& cfg) {
  common::DegeneracyReport report;  // eigvals 0, eigvecs I, nothing flagged
  if (lower_triangle_finite(hessian)) {
    // Reads the lower triangle only; eigenvalues ascending, unit eigenvectors.
    const Eigen::SelfAdjointEigenSolver<common::Mat6> solver(hessian, Eigen::ComputeEigenvectors);
    if (solver.info() == Eigen::Success) {
      report.eigvals = solver.eigenvalues();
      report.eigvecs = solver.eigenvectors();
      for (std::size_t k = 0; k < report.degenerate.size(); ++k) {
        report.degenerate[k] =
            cfg.enabled && report.eigvals(static_cast<int>(k)) < cfg.eig_threshold;
      }
      return report;
    }
  }
  // Non-finite H or solver failure: no usable information, every direction degenerate.
  for (bool& flag : report.degenerate) {
    flag = cfg.enabled;
  }
  return report;
}

double marginal_information(const common::DegeneracyReport& report, const common::Vec6& u) {
  // u^T H^-1 u = sum_k (v_k . u)^2 / lambda_k over the eigen-decomposition H = V diag(lambda) V^T
  // (degeneracy.md section 8). Eigenvalues at or below kNullRelative x the largest are the null
  // space (a rank-deficient H gives round-off of either sign there): a component along it larger
  // than kNullRelative x |u|^2 makes the variance unbounded (0), a smaller one is round-off.
  const double lambda_max = report.eigvals.maxCoeff();
  const double u2 = u.squaredNorm();
  if (!(lambda_max > 0.0) || !std::isfinite(lambda_max) || !(u2 > 0.0)) {
    return 0.0;
  }
  const double null_eig = kNullRelative * lambda_max;
  double var = 0.0;
  for (int k = 0; k < kDim; ++k) {
    const double c = report.eigvecs.col(k).dot(u);
    const double c2 = c * c;
    const double lambda = report.eigvals(k);
    if (!std::isfinite(lambda)) {
      return 0.0;
    }
    if (!(lambda > null_eig)) {
      if (c2 > kNullRelative * u2) {
        return 0.0;
      }
      continue;
    }
    var += c2 / lambda;
  }
  return var > 0.0 ? 1.0 / var : 0.0;
}

common::Vec6 project_update(const common::Vec6& delta, const common::DegeneracyReport& report) {
  if (!any_flagged(report)) {
    return delta;  // exact identity (feature disabled or well-constrained)
  }
  // delta' = V diag(m) V^T delta = delta - sum_{k flagged} v_k (v_k^T delta).
  common::Vec6 out = delta;
  for (std::size_t k = 0; k < report.degenerate.size(); ++k) {
    if (report.degenerate[k]) {
      const common::Vec6 v = report.eigvecs.col(static_cast<int>(k));
      out -= v.dot(delta) * v;
    }
  }
  return out;
}

common::PoseCovariance relax_covariance(const common::PoseCovariance& cov,
                                        const common::DegeneracyReport& report,
                                        const DegeneracyConfig& cfg) {
  if (!any_flagged(report)) {
    return cov;  // exact identity (feature disabled or well-constrained)
  }
  // A = V diag(s) V^T = I + sum_{k flagged} (sqrt(f) - 1) v_k v_k^T, cov' = A cov A^T (A =
  // `inflate`). Precondition relax_factor >= 1 is validated by load_config (no checks on the hot
  // path).
  const double scale_minus_one = std::sqrt(cfg.relax_factor) - 1.0;
  common::Mat6 inflate = common::Mat6::Identity();
  for (std::size_t k = 0; k < report.degenerate.size(); ++k) {
    if (report.degenerate[k]) {
      const common::Vec6 v = report.eigvecs.col(static_cast<int>(k));
      inflate.noalias() += scale_minus_one * v * v.transpose();
    }
  }
  const common::Mat6 inflated = inflate * cov * inflate.transpose();
  // Exact symmetry: (i, j) and (j, i) are the same floating-point sum.
  const common::PoseCovariance out = 0.5 * (inflated + inflated.transpose());
  return out;
}

common::PoseCovariance unobserved_covariance(const common::PoseCovariance& cov,
                                             const common::DegeneracyReport& report,
                                             const DegeneracyConfig& cfg,
                                             double var_trans_m2,
                                             double var_rot_rad2,
                                             double c) {
  if (!any_flagged(report)) {
    return cov;  // exact identity (feature disabled or well-constrained)
  }
  // P_F = sum_{k flagged} v_k v_k^T, P = I - P_F; U = P_F D P_F with
  // D = diag(var_trans_m2 I3, var_rot_rad2 I3), each eigenvalue of U on F floored at
  // relax_factor * c / eig_threshold (docs/derivations/scan_match.md section 6). U does not depend
  // on the basis the solver picked inside a repeated flagged eigenvalue.
  common::Mat6 p_flagged = common::Mat6::Zero();
  int n_flagged = 0;
  for (std::size_t k = 0; k < report.degenerate.size(); ++k) {
    if (report.degenerate[k]) {
      const common::Vec6 v = report.eigvecs.col(static_cast<int>(k));
      p_flagged.noalias() += v * v.transpose();
      ++n_flagged;
    }
  }
  common::Vec6 d_diag;
  d_diag << var_trans_m2, var_trans_m2, var_trans_m2, var_rot_rad2, var_rot_rad2, var_rot_rad2;
  const common::Mat6 d = d_diag.asDiagonal();
  common::Mat6 unobserved = common::Mat6::Zero();
  if (n_flagged == 1) {
    // One flagged vector: U = (v^T D v) v v^T directly (no eigen-solve).
    for (std::size_t k = 0; k < report.degenerate.size(); ++k) {
      if (report.degenerate[k]) {
        const common::Vec6 v = report.eigvecs.col(static_cast<int>(k));
        const double var = floored_variance(v.dot(d * v), cfg, c);
        unobserved.noalias() += var * (v * v.transpose());
      }
    }
  } else {
    const common::Mat6 u_raw = p_flagged * d * p_flagged;
    const common::Mat6 u_sym = 0.5 * (u_raw + u_raw.transpose());
    const Eigen::SelfAdjointEigenSolver<common::Mat6> es(u_sym, Eigen::ComputeEigenvectors);
    for (int i = 0; i < kDim; ++i) {
      const common::Vec6 w = es.eigenvectors().col(i);
      const common::Vec6 w_f = p_flagged * w;
      if (w_f.squaredNorm() > 0.5) {  // w lies in the flagged subspace (unit vector)
        const double var = floored_variance(es.eigenvalues()(i), cfg, c);
        unobserved.noalias() += var * (w * w.transpose());
      }
    }
  }
  const common::Mat6 projector = common::Mat6::Identity() - p_flagged;
  const common::Mat6 observed = projector * cov * projector;
  const common::Mat6 out = observed + unobserved;
  // Exact symmetry: (i, j) and (j, i) are the same floating-point sum.
  const common::PoseCovariance sym = 0.5 * (out + out.transpose());
  return sym;
}

}  // namespace mss::degeneracy
