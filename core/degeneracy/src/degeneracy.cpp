#include "mss/degeneracy/degeneracy.hpp"

#include <cmath>
#include <cstddef>

#include <Eigen/Eigenvalues>

// Derivation: docs/derivations/degeneracy.md. Tangent [rho, phi] of the pose H was built
// for (right perturbation); no frame conversion here. Hot path: fixed-size Eigen only.
namespace mss::degeneracy {
namespace {

constexpr int kDim = 6;

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

}  // namespace mss::degeneracy
