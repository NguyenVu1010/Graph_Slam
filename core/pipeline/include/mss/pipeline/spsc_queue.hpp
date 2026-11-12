#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace mss::pipeline {

/// Fixed-capacity FIFO between exactly one producer thread and one consumer thread
/// (ADR-033 decision 2). Lock-free and allocation-free: the elements live inline in a
/// std::array, and the producer and consumer each own one monotonically increasing index.
///
/// Full policy: push() rejects the new element and returns false; the caller counts it.
/// (The producer cannot drop the oldest element: that would make it write the consumer's
/// index.) Where newest-wins matters (scans), the consumer drains the queue and keeps the
/// newest.
///
/// Threading: push() from the producer thread only, pop() from the consumer thread only.
/// size() and empty() may be called from either and are a snapshot. T must be
/// default-constructible and copy-assignable.
template <class T, std::size_t N>
class SpscQueue {
  static_assert(N > 0, "SpscQueue capacity must be positive");

 public:
  static constexpr std::size_t capacity() { return N; }

  /// Producer only. Copies `v` in. Returns false (queue unchanged) when full.
  bool push(const T& v) {
    const std::size_t w = write_.load(std::memory_order_relaxed);
    if (w - read_.load(std::memory_order_acquire) == N) {
      return false;
    }
    data_[w % N] = v;
    write_.store(w + 1, std::memory_order_release);
    return true;
  }

  /// Consumer only. Copies the oldest element into `out`. Returns false (and leaves `out`)
  /// when empty.
  bool pop(T& out) {
    const std::size_t r = read_.load(std::memory_order_relaxed);
    if (r == write_.load(std::memory_order_acquire)) {
      return false;
    }
    out = data_[r % N];
    read_.store(r + 1, std::memory_order_release);
    return true;
  }

  /// Number of queued elements at some instant during the call.
  std::size_t size() const {
    const std::size_t r = read_.load(std::memory_order_acquire);
    return write_.load(std::memory_order_acquire) - r;
  }
  bool empty() const { return size() == 0; }

 private:
  // Separate cache lines: the producer writes write_, the consumer writes read_.
  alignas(64) std::atomic<std::size_t> write_{0};
  alignas(64) std::atomic<std::size_t> read_{0};
  alignas(64) std::array<T, N> data_{};
};

}  // namespace mss::pipeline
