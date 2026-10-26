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
bool lower_triangle_finite(const common::Mat6& H) {
  for (int j = 0; j < kDim; ++j) {
    for (int i = j; i < kDim; ++i) {
      if (!std::isfinite(H(i, j))) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

common::DegeneracyReport analyze(const common::Mat6& H, const DegeneracyConfig& cfg) {
  common::DegeneracyReport report;  // eigvals 0, eigvecs I, nothing flagged
  if (lower_triangle_finite(H)) {
    // Reads the lower triangle only; eigenvalues ascending, unit eigenvectors.
    const Eigen::SelfAdjointEigenSolver<common::Mat6> solver(H, Eigen::ComputeEigenvectors);
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

}  // namespace mss::degeneracy
