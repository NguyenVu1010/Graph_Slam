#pragma once

#include "mss2d/small_matrix.hpp"

namespace mss2d {

/// Eigen-decomposition of a symmetric 3x3 matrix: values ascending, vectors as the
/// matching columns (unit, orthogonal).
struct SymEigen3 {
  float values[3]{0, 0, 0};
  Mat3f vectors;
};

/// Cyclic Jacobi rotations (at most 16 sweeps; 3x3 converges in about 4). Reads only the
/// upper triangle. Deterministic, no allocation.
SymEigen3 sym_eigen3(const Mat3f& a);

/// Degeneracy of a scan-matching Hessian, same rule as core/degeneracy (ADR-004,
/// docs/derivations/degeneracy.md): direction k is degenerate iff eig_threshold > 0 and
/// lambda_k < eig_threshold.
struct DegeneracyReport {
  SymEigen3 eig;
  bool degenerate[3]{false, false, false};
  int n_degenerate = 0;
};

DegeneracyReport analyse_degeneracy(const Mat3f& h, float eig_threshold);

}  // namespace mss2d
