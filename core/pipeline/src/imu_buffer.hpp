#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/pipeline/bounded_queue.hpp"

namespace mss::pipeline::detail {

/// Ring of the most recent IMU samples (capacity kMaxImuQueue), strictly increasing in t.
/// Storage is allocated once in the constructor; no method allocates afterwards.
/// Internal to the pipeline (not installed). Single-threaded: owned by EstimatorCore.
class ImuBuffer {
 public:
  ImuBuffer();

  /// Appends `s`. Returns false and stores nothing when `s.t <= back().t`. When the
  /// buffer is full the oldest sample is dropped first (the caller checks full() before
  /// pushing when it must see that sample).
  bool push(const common::ImuSample& s);

  [[nodiscard]] std::size_t size() const { return size_; }
  [[nodiscard]] bool empty() const { return size_ == 0; }
  [[nodiscard]] bool full() const { return size_ == data_.size(); }
  static constexpr std::size_t capacity() { return kMaxImuQueue; }

  /// Sample i, 0 = oldest. Precondition: i < size().
  [[nodiscard]] const common::ImuSample& at(std::size_t i) const {
    return data_[(head_ + i) % data_.size()];
  }
  /// Precondition: !empty().
  [[nodiscard]] const common::ImuSample& front() const { return at(0); }
  [[nodiscard]] const common::ImuSample& back() const { return at(size_ - 1); }
  /// Removes the oldest sample. Precondition: !empty().
  void pop_front();

  /// Sample at time `t`, acc and gyro linear in time between the bracketing samples
  /// (docs/derivations/pipeline.md section 1). A raw sample at exactly `t` is returned
  /// as is. nullopt when `t` lies outside [front().t, back().t] or the buffer is empty.
  [[nodiscard]] std::optional<common::ImuSample> interpolate(common::Timestamp t) const;

  /// Index of the first sample with `at(i).t > t` (size() when there is none).
  [[nodiscard]] std::size_t first_after(common::Timestamp t) const;

  /// Copies the samples with `t0 <= t <= t1` into `out[0, n)` and returns n.
  std::size_t copy_range(common::Timestamp t0,
                         common::Timestamp t1,
                         std::array<common::ImuSample, kMaxImuQueue>& out) const;

 private:
  std::vector<common::ImuSample> data_;  // kMaxImuQueue slots, allocated once
  std::size_t head_ = 0;                 // slot of the oldest sample
  std::size_t size_ = 0;
};

}  // namespace mss::pipeline::detail
