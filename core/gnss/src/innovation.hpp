#pragma once

// Internal (not installed) GNSS innovation helpers. gnss.hpp has no public place for the
// chi-square innovation test and the GNSS position residual yet; they live here in
// mss::gnss::detail so promoting them to the public header later is a move, not a
// rewrite (see docs/superpowers/plans/2026-10-26-gnss.md, Contract issue 1).

#include <Eigen/Core>

#include "mss/common/result.hpp"
#include "mss/common/types.hpp"

namespace mss::gnss::detail {

/// Chi-square quantile, 3 DoF, q = 0.95 (docs/derivations/gnss.md section 3). Becomes
/// GnssConfig::chi2_gate (YAML gnss.chi2_gate) once Contract issue 1 is agreed.
constexpr double kChi2Gate3Dof95 = 7.814727903251178;

/// GNSS position residual and its Jacobian (docs/derivations/gnss.md section 2).
///   Residual: r = p_e_g - T_e_w * (T_w_b * p_b_g)   (3x1, m, expressed in e).
///   Variable: T_w_b, right perturbation T_w_b <- T_w_b * Exp(xi), xi = [rho, phi].
///   J_r_xi = [ -R_e_w R_w_b ,  R_e_w R_w_b skew(p_b_g) ]   (3x6).
/// Frames: p_e_g antenna in e; T_e_w maps w to e; T_w_b body in w; p_b_g lever arm in b.
/// `J_r_xi` may be nullptr. No allocation.
common::Vec3 gnss_residual(const common::Vec3& p_e_g,
                           const common::Pose& T_e_w,
                           const common::Pose& T_w_b,
                           const common::Vec3& p_b_g,
                           Eigen::Matrix<double, 3, 6>* J_r_xi);

/// Normalised innovation squared r^T S^-1 r (S in m^2, r in m, both in e).
/// Errors: "non-finite innovation" (r or S non-finite), "innovation covariance not
/// positive definite" (LLT fails). No allocation.
common::Result<double> innovation_nis(const common::Vec3& r_e, const common::Mat3& S_e);

/// Chi-square gate: ok when NIS <= threshold. Errors: those of innovation_nis, then
/// "chi-square gate exceeded". No allocation.
common::Status chi_square_gate(const common::Vec3& r_e, const common::Mat3& S_e, double threshold);

}  // namespace mss::gnss::detail
