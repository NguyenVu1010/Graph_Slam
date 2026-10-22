#pragma once

#include <array>
#include <cstddef>

namespace mss::pipeline {

/// Compile-time capacities of the estimator queues. The YAML depths (realtime.*) are
/// runtime limits enforced by Estimator and must not exceed these (checked in the
/// Estimator constructor).
constexpr std::size_t kMaxImuQueue = 4096;  // IMU samples
constexpr std::size_t kMaxScanQueue = 16;   // scans
constexpr std::size_t kMaxAuxQueue = 256;   // GNSS fixes or wheel samples, per queue

/// Fixed-capacity FIFO ring buffer stored inline in a std::array: no allocation ever.
/// When full, push() overwrites the oldest element and returns false (drop-oldest policy,
/// ADR-005). T must be default-constructible and copy-assignable.
///
/// Threading: no mutex; single-threaded use only in this phase.
/// TODO(ADR-005): a thread-safe variant (one producer, one consumer) once the estimator
/// threads exist.
template <class T, std::size_t N>
class BoundedQueue {
  static_assert(N > 0, "BoundedQueue capacity must be positive");

 public:
  static constexpr std::size_t capacity() { return N; }
  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }

  /// Appends `v`. Returns true, or false when the queue was full and its oldest element
  /// was overwritten.
  bool push(const T& v) {
    const bool had_room = size_ < N;
    if (!had_room) {
      head_ = next(head_);  // drop the oldest
      --size_;
    }
    data_[(head_ + size_) % N] = v;
    ++size_;
    return had_room;
  }

  /// Removes the oldest element into `out`. Returns false (and leaves `out`) when empty.
  bool pop(T& out) {
    if (size_ == 0) {
      return false;
    }
    out = data_[head_];
    head_ = next(head_);
    --size_;
    return true;
  }

  /// Oldest element. Precondition: !empty().
  const T& front() const { return data_[head_]; }

 private:
  static std::size_t next(std::size_t i) { return i + 1 == N ? 0 : i + 1; }

  std::array<T, N> data_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};

}  // namespace mss::pipeline
