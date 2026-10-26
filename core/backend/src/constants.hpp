#pragma once

// Internal backend constants shared by graph.cpp and the backend tests.

namespace mss::backend::detail {

// Robust kernel width for loop factors (backend_factors.md section 5, ADR-015):
// sqrt(chi2_6(0.95)) = sqrt(12.5916), in whitened (Mahalanobis) units. A
// backend.loop_cauchy_k YAML key comes with the loop module.
inline constexpr double kLoopCauchyK = 3.5485;

}  // namespace mss::backend::detail
