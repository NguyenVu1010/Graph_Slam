#pragma once

// Internal backend constants shared by graph.cpp and the backend tests.

namespace mss::backend::detail {

// Robust kernel width for loop factors (backend_factors.md section 5, ADR-015):
// sqrt(chi2_6(0.95)) = sqrt(12.5916), in whitened (Mahalanobis) units. A quantile, not a
// tunable, so no YAML key (ADR-026 decision 3).
inline constexpr double kLoopCauchyK = 3.5485;

// Gravity-factor consistency check (backend_factors.md section 6c, ADR-020): smallest
// eigenvalue of the whitened innovation covariance S = (I + J P_rest J^T)^-1 (dimensionless,
// in (0, 1]: the factor variance over the total innovation variance, per direction) below
// which the statistic is not evaluated. Below 1e-9 the rest of the graph is more than 1e9
// times less certain (in variance) than the factor about what the factor measures: the
// factor is in effect the only information there, and nothing can contradict it. The
// scenes measure 0.013 to 0.66 with the check on (LiDAR-only, box and corridor component
// tests, imu_only_observability.md 10g). A
// bound on a ratio, not a tunable.
inline constexpr double kMinGravityInnovationEigen = 1e-9;

}  // namespace mss::backend::detail
