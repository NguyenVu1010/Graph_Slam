#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mss/common/timestamp.hpp"

namespace mss::pipeline {

/// Input queues of the estimator, in tie-break order: on equal keys a lower value is
/// processed first (IMU, wheel, GNSS, scan), see Estimator::run_once().
enum class Source : std::uint8_t { kImu = 0, kWheel = 1, kGnss = 2, kScan = 3, kNone = 4 };

constexpr std::size_t kNumSources = 4;

/// Head of one FIFO queue as seen by the k-way merge. `key` is the ordering time: `t` for
/// IMU, wheel and GNSS, the scan end time `t_start + max(t_offset_ns)` for a scan.
struct QueueHead {
  bool present = false;  ///< false when the queue is empty
  common::Timestamp key{};
};

/// One step of the k-way merge: the source (indexed by Source) whose head has the
/// smallest key; ties go to the lower Source value. Returns Source::kNone when every
/// queue is empty. Hot path: yes, no allocation.
inline Source select_next(const std::array<QueueHead, kNumSources>& heads) noexcept {
  Source best = Source::kNone;
  common::Timestamp best_key{};
  for (std::size_t i = 0; i < kNumSources; ++i) {
    // Strictly smaller wins, so the first source in tie-break order keeps a tie.
    if (heads[i].present && (best == Source::kNone || heads[i].key < best_key)) {
      best = static_cast<Source>(i);
      best_key = heads[i].key;
    }
  }
  return best;
}

}  // namespace mss::pipeline
