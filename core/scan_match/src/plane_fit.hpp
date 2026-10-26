#pragma once

#include <optional>

#include "mss/common/types.hpp"
#include "mss/map/voxel_map.hpp"

// Internal to mss_scan_match (not installed). Derivation: docs/derivations/scan_match.md,
// section 3.
namespace mss::scan_match::detail {

struct PlaneFit {
  // m, centroid of the neighbours, frame w.
  common::Vec3 q_w = common::Vec3::Zero();
  // Unit normal in w: eigenvector of the smallest eigenvalue of the scatter matrix.
  common::Vec3 n_w = common::Vec3::UnitZ();
  // m, RMS point-to-plane distance of the neighbours = sqrt(lambda_min / n).
  double rms_m = 0.0;
};

// Least-squares plane through `buf.p_w[0, buf.size)`. nullopt when buf.size < 3.
// Hot path: no allocation, no logging, no exceptions (fixed-size Eigen only).
std::optional<PlaneFit> fit_plane(const map::NeighborBuffer& buf);

}  // namespace mss::scan_match::detail
