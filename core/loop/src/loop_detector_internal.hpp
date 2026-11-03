#pragma once

#include <cstdint>

#include "mss/loop/loop_detector.hpp"

// Internal to the loop module (not under include/, not part of the contract): diagnostics that
// tests read through a private include path (reviews N2-3, R3-1).
namespace mss::loop {

/// Read access to a detector's internal diagnostics (a friend of LoopDetector).
struct DetectorInternals {
  /// Scan Context descriptors `d` has computed since construction (every fill_descriptor call:
  /// the query's each search, and with place_search each cache miss or refresh). Per detector,
  /// so detectors do not count each other's work; call it on the thread that owns `d`.
  static std::uint64_t descriptor_fill_count(const LoopDetector& d);
};

namespace detail {
inline std::uint64_t descriptor_fill_count(const LoopDetector& d) {
  return DetectorInternals::descriptor_fill_count(d);
}
}  // namespace detail

}  // namespace mss::loop
