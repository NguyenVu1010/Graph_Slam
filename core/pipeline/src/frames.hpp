#pragma once

#include "mss/common/types.hpp"

namespace mss::pipeline::detail {

/// Scan-match covariance re-expressed from the tangent of T_w_l to that of T_w_b
/// (docs/derivations/pipeline.md section 3): `Ad(T_b_l) cov_l Ad(T_b_l)^T`.
/// Units: m^2 / rad^2 blocks, [rho, phi]. No allocation.
common::PoseCovariance lidar_cov_in_body(const common::PoseCovariance& cov_l,
                                         const common::Pose& T_b_l);

/// Degeneracy report from the l tangent to the b tangent: eigvecs `Ad(T_b_l) V_l`,
/// eigenvalues and flags unchanged (pipeline.md section 3). Only the flagged directions are
/// meaningful afterwards: the columns are not unit-length, not orthogonal and not
/// eigenvectors of the Hessian in b (diagnostics only, graph.hpp). No allocation.
common::DegeneracyReport degeneracy_in_body(const common::DegeneracyReport& r_l,
                                            const common::Pose& T_b_l);

}  // namespace mss::pipeline::detail
