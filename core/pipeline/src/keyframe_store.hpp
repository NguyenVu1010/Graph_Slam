#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"

namespace mss::pipeline::detail {

/// One stored keyframe (ADR-016): its id, time and the downsampled cloud in l
/// (deskewed to `t`).
struct KeyframeEntry {
  common::KeyId id = 0;
  common::Timestamp t{};
  common::ScanFrame cloud_l;
};

/// ADR-016 keyframe store: a ring of `max_keyframes` slots allocated once; the oldest
/// keyframe is evicted when a new one arrives and the store is full. The map cache is
/// rebuilt from these clouds after a pose correction.
/// Allocation: add() copies with ScanFrame::copy_from and re-reserves a slot (clear()
/// then reserve(cloud.size())) only when its capacity is too small; otherwise none.
/// Internal to the pipeline. Single-threaded: owned by EstimatorCore.
class KeyframeStore {
 public:
  /// Throws std::invalid_argument when `max_keyframes == 0`.
  explicit KeyframeStore(std::size_t max_keyframes);

  /// Stores a copy of `cloud_l` as keyframe `id` at `t`.
  /// Errors: "keyframe id not increasing" when `id <=` the newest stored id (nothing
  /// stored).
  common::Status add(common::KeyId id, common::Timestamp t, const common::ScanFrame& cloud_l);

  std::size_t size() const { return size_; }
  std::size_t capacity() const { return slots_.size(); }
  /// Keyframe i, 0 = oldest kept. Precondition: i < size().
  const KeyframeEntry& at(std::size_t i) const { return slots_[(head_ + i) % slots_.size()]; }
  /// Keyframe `id`, or nullptr when it was evicted or never added. O(log size()).
  const KeyframeEntry* find(common::KeyId id) const;
  /// Keyframes evicted so far.
  std::uint64_t n_evicted() const { return n_evicted_; }

 private:
  std::vector<KeyframeEntry> slots_;
  std::size_t head_ = 0;  // slot of the oldest kept keyframe
  std::size_t size_ = 0;
  std::uint64_t n_evicted_ = 0;
};

}  // namespace mss::pipeline::detail
