#pragma once

#include <optional>

#include "mss/common/types.hpp"
#include "mss/map/voxel_map.hpp"

// Internal to mss_scan_match (not installed). Derivation: docs/derivations/scan_match.md,
// section 3.
namespace mss::scan_match::detail {

// A plane needs at least 3 points (geometry, not a tunable). Shared by the fit and the
// correspondence gate in scan_matcher.cpp.
constexpr int kMinPlanePoints = 3;

struct PlaneFit {
  // m, centroid of the neighbours, frame w.
  common::Vec3 q_w = common::Vec3::Zero();
  // Unit normal in w: eigenvector of the smallest eigenvalue of the scatter matrix.
  common::Vec3 n_w = common::Vec3::UnitZ();
  // m, RMS point-to-plane distance of the neighbours = sqrt(lambda_min / n).
  double rms_m = 0.0;
  // m^2, eigenvalues of the scatter matrix S (lambda_min <= lambda_mid <= lambda_max).
  double lambda_min_m2 = 0.0;
  double lambda_mid_m2 = 0.0;
  double lambda_max_m2 = 0.0;
};

// Least-squares plane through `buf.p_w[0, buf.size)`. nullopt when buf.size < 3.
// Hot path: no allocation, no logging, no exceptions (fixed-size Eigen only).
std::optional<PlaneFit> fit_plane(const map::NeighborBuffer& buf);

// Planarity gate (derivation section 3, ADR-007 amendment): with `min_eig_ratio` > 0 the
// neighbourhood is rejected when lambda_min > min_eig_ratio * lambda_mid (a two-face edge or
// a corner); `min_eig_ratio` == 0 turns the gate off (always true). Written without a
// division, so a collinear neighbourhood (lambda_min = lambda_mid = 0) passes.
// Hot path: no allocation, no logging, no exceptions.
bool passes_planarity(const PlaneFit& fit, double min_eig_ratio);

// Collinearity gate (derivation section 3, ADR-007 amendment): with `min_mid_ratio` > 0 the
// neighbourhood is rejected unless lambda_mid > min_mid_ratio * lambda_max (points on one
// line, such as a single LiDAR ring, have lambda_mid ~ 0 and an arbitrary normal; identical
// points, all eigenvalues 0, are rejected too); `min_mid_ratio` == 0 turns it off.
// Hot path: no allocation, no logging, no exceptions.
bool passes_collinearity(const PlaneFit& fit, double min_mid_ratio);

}  // namespace mss::scan_match::detail
