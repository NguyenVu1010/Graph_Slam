#pragma once

#include "mss/loop/config.hpp"

namespace mss::loop {

/// Loop-closure detection and verification (phase 2, disabled by default via
/// LoopConfig::enabled). Placeholder contract: only the constructor is fixed now; the
/// query/verify API (producing common::LoopConstraint, tests L-01..L-03) is designed by
/// the loop module in phase 2 with its own ADR.
///
/// Ownership/threading: single-threaded; owned by pipeline::Estimator, runs on the loop
/// thread (ADR-005).
class LoopDetector {
 public:
  /// Errors: throws std::invalid_argument when `enabled` and (`search_radius_m <= 0` or
  ///   `min_time_gap_s < 0`). Hot path: no. Tests: L-01..L-03 (phase 2).
  explicit LoopDetector(const LoopConfig& cfg);
};

}  // namespace mss::loop
