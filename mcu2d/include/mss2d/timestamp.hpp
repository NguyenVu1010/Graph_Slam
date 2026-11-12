#pragma once

#include <cstdint>

namespace mss2d {

// Mirrors mss::common::Timestamp (core/common/timestamp.hpp) on purpose: mcu2d does not link
// core/ (ADR-034). Sensor-clock time in integer nanoseconds; never compare floating point.
struct Timestamp {
  std::int64_t ns{0};

  constexpr bool operator==(const Timestamp& o) const { return ns == o.ns; }
  constexpr bool operator!=(const Timestamp& o) const { return ns != o.ns; }
  constexpr bool operator<(const Timestamp& o) const { return ns < o.ns; }
  constexpr bool operator<=(const Timestamp& o) const { return ns <= o.ns; }
  constexpr bool operator>(const Timestamp& o) const { return ns > o.ns; }
  constexpr bool operator>=(const Timestamp& o) const { return ns >= o.ns; }
};

constexpr std::int64_t kNanosecondsPerSecond = 1000000000;

}  // namespace mss2d
