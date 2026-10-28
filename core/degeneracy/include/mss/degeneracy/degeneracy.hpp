#pragma once

#include "mss/common/types.hpp"
#include "mss/degeneracy/config.hpp"

// Degeneracy detection and handling for scan matching (D-01..D-03).
//
// Tangent convention for every function here: the [rho, phi] right perturbation of the
// pose the Hessian was built for. For the scan matcher that is T_w_l, i.e. LiDAR frame l
// (MatchResult::H). Callers: scan_match::ScanMatcher::align calls analyze +
// project_update on every Gauss-Newton iteration (when enabled) and unobserved_covariance
// on the final MatchResult::cov; the pipeline may call analyze on MatchResult::H for the
// report but must NOT change the covariance again. The pipeline then converts to body
// frame b: covariance with
// transform_covariance(cov, T_b_l), eigenvectors with se3_adj(T_b_l) * eigvecs.col(k)
// (lie.hpp). Mixing frames between H, delta, cov and report is a caller bug.
//
// Free functions, stateless, thread-compatible. Feature flag: DegeneracyConfig::enabled
// (struct default false; see config/default.yaml). When disabled, analyze()
// flags nothing, which makes project_update(), relax_covariance() and
// unobserved_covariance() exact identities. With nothing flagged the same holds when enabled.
namespace mss::degeneracy {

/// Eigen-decomposition of the scan-matching Hessian and per-direction degeneracy flags.
/// `eigvals` ascending; `eigvecs.col(k)` unit-norm eigenvector of `eigvals(k)` (sign
/// arbitrary); `degenerate[k] = cfg.enabled && eigvals(k) < cfg.eig_threshold`.
/// Frames: tangent of H (see file comment). Units: eigenvalues in the units of H
///   (absolute, not normalised; mixed m/rad as in MatchResult::H).
/// Preconditions: H symmetric (only the lower triangle is read).
/// Errors: none. A non-finite H yields eigvals = 0, eigvecs = I and, when enabled,
///   every direction flagged degenerate.
/// Hot path: yes, per scan. Fixed-size Eigen only; no allocation, logging or exceptions.
/// Tests: D-01 (corridor flagged, direction within 5 deg of the axis), D-02 (6-sided room
///   not flagged).
common::DegeneracyReport analyze(const common::Mat6& H, const DegeneracyConfig& cfg);

/// Removes the components of a Gauss-Newton update along degenerate directions:
/// `delta' = V * diag(m) * V^T * delta`, `m_k = 0` if `report.degenerate[k]` else 1,
/// `V = report.eigvecs`. With no flags the result equals `delta` exactly.
/// Frames: `delta` in the same tangent as the H given to analyze(). Units: [m, rad].
/// Errors: none. Hot path: yes (per iteration); no allocation, logging or exceptions.
/// Tests: D-03 (update along the degenerate direction is 0).
common::Vec6 project_update(const common::Vec6& delta, const common::DegeneracyReport& report);

/// Inflates a covariance along degenerate directions by `cfg.relax_factor`:
/// `cov' = A * cov * A^T`, `A = V * diag(s) * V^T`, `s_k = sqrt(relax_factor)` if
/// `report.degenerate[k]` else 1. Hence `v_k^T cov' v_k = relax_factor * v_k^T cov v_k`
/// for every degenerate eigenvector `v_k`, and cov' is symmetric (symmetrised exactly).
/// With no flags the result equals `cov` exactly.
/// Callers: none in the pipeline since PR 4b; scan_match uses unobserved_covariance()
///   instead (docs/derivations/scan_match.md section 6). Kept for callers that want a
///   multiplicative inflation.
/// Frames: `cov` in the same tangent as the H given to analyze() (l for scan matching,
///   before the pipeline transforms it to b). Units: m^2 / rad^2 blocks.
/// Preconditions: `cfg.relax_factor >= 1` (validated by load_config).
/// Errors: none. Hot path: yes (per scan); no allocation, logging or exceptions.
/// Tests: D-03 (covariance along the degenerate direction grows >= relax_factor times).
common::PoseCovariance relax_covariance(const common::PoseCovariance& cov,
                                        const common::DegeneracyReport& report,
                                        const DegeneracyConfig& cfg);

/// Replaces the covariance along degenerate directions by a fixed "unobserved" variance:
/// `cov' = P * cov * P + U'`, symmetrised exactly, with `P_F = sum_{k flagged} v_k v_k^T`,
/// `P = I - P_F` (projector onto the observed directions), `D = diag(var_trans_m2 * I3,
/// var_rot_rad2 * I3)` in [rho, phi] order, `U = P_F D P_F` and `U'` = U with each of its
/// eigenvalues `u` on the flagged subspace replaced by `max(u, cfg.relax_factor * c /
/// cfg.eig_threshold)`; no floor when `eig_threshold <= 0`. With one flagged vector
/// `U' = max(v^T D v, floor) v v^T` (no eigen-solve).
/// U does not depend on the basis the eigen-solver picked inside a repeated flagged eigenvalue.
/// The floor is the pre-PR-4b relaxed value: an unobserved direction is never more confident
/// than that, nor than an observed direction of the regularised system (eigenvalue >=
/// eig_threshold, variance <= c / eig_threshold). If the configured variances are below the floor,
/// the result is the pre-PR-4b relaxed covariance. Every entry `v_i^T cov' v_j` between unflagged
/// directions equals `v_i^T cov v_j`, and the flagged subspace is decoupled from the observed one
/// (exactly `var_trans_m2` along a pure translation axis when the floor does not apply). With
/// no flags the result equals `cov` exactly.
/// `c`: the scale of the regularised inverse in align(), `cov_scale * max(sigma_r^2,
///   min_residual_sigma_m^2)` (m^2).
/// Caller: scan_match::ScanMatcher::align applies it to MatchResult::cov (when enabled) with
///   scan_match.unobserved_var_m2 / unobserved_var_rad2, so that cov is final; the pipeline
///   must not change it again (docs/derivations/scan_match.md section 6).
/// Frames: as relax_covariance. Units: m^2 / rad^2 blocks; var_trans_m2 in m^2,
///   var_rot_rad2 in rad^2.
/// Preconditions: both variances > 0, c >= 0, relax_factor >= 1 (validated by load_config /
///   ScanMatcher).
/// Errors: none. Hot path: yes (per scan); no allocation, logging or exceptions.
/// Tests: D-03 (corridor axis variance equals unobserved_var_m2, observed unchanged),
///   UnobservedCovariance.FloorAppliesWhenConfiguredVarianceIsSmaller,
///   UnobservedCovariance.RepeatedFlaggedEigenvalueBasisInvariant.
common::PoseCovariance unobserved_covariance(const common::PoseCovariance& cov,
                                             const common::DegeneracyReport& report,
                                             const DegeneracyConfig& cfg,
                                             double var_trans_m2,
                                             double var_rot_rad2,
                                             double c);

}  // namespace mss::degeneracy
