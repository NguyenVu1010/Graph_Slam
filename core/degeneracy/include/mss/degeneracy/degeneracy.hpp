#pragma once

#include "mss/common/types.hpp"
#include "mss/degeneracy/config.hpp"

// Degeneracy detection and handling for scan matching (D-01..D-03).
//
// Tangent convention for every function here: the [rho, phi] right perturbation of the
// pose the Hessian was built for. For the scan matcher that is T_w_l, i.e. LiDAR frame l
// (MatchResult::H). Callers: scan_match::ScanMatcher::align calls analyze +
// project_update on every Gauss-Newton iteration (when enabled) and relax_covariance on
// the final MatchResult::cov; the pipeline may call analyze on MatchResult::H for the
// report but must NOT relax the covariance again. The pipeline then converts to body
// frame b: covariance with
// transform_covariance(cov, T_b_l), eigenvectors with se3_adj(T_b_l) * eigvecs.col(k)
// (lie.hpp). Mixing frames between H, delta, cov and report is a caller bug.
//
// Free functions, stateless, thread-compatible. Feature flag: DegeneracyConfig::enabled
// (default false). When disabled, analyze() flags nothing, which makes project_update()
// and relax_covariance() exact identities, so default behaviour is unchanged.
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
/// Caller: scan_match::ScanMatcher::align applies it to MatchResult::cov (when enabled),
///   so that cov is final. The relaxed covariance is the only degeneracy mitigation in
///   the factor; the pipeline passes the DegeneracyReport to backend for diagnostics only
///   (see graph.hpp) and must not call relax_covariance again.
/// Frames: `cov` in the same tangent as the H given to analyze() (l for scan matching,
///   before the pipeline transforms it to b). Units: m^2 / rad^2 blocks.
/// Preconditions: `cfg.relax_factor >= 1` (validated by load_config).
/// Errors: none. Hot path: yes (per scan); no allocation, logging or exceptions.
/// Tests: D-03 (covariance along the degenerate direction grows >= relax_factor times).
common::PoseCovariance relax_covariance(const common::PoseCovariance& cov,
                                        const common::DegeneracyReport& report,
                                        const DegeneracyConfig& cfg);

}  // namespace mss::degeneracy
