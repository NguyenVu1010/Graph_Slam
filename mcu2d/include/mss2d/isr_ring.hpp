#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace mss2d {

// Single-producer single-consumer ring: the producer is an ISR (or, on the host, one
// thread), the consumer is the main loop. Fixed capacity N, no allocation, no locks.
//
// Why this is correct on a single-core Cortex-M: the ISR preempts the main loop but never
// the other way round, and each index has exactly one writer (head_: producer, tail_:
// consumer). The producer writes the slot, then publishes it with a release store of head_;
// the consumer reads head_ with acquire before touching the slot, so it never sees a
// half-written element. On one core there is no cross-core reordering, but the compiler may
// still reorder the slot write and the index store; release/acquire forbid that, and on
// Armv7-M the release store also emits `dmb` where needed. The same holds on a multi-core
// host, which is what the TSAN stress test exercises. Indices and the drop counter are 32
// bit with a plain load + store (single writer), so no read-modify-write is required.
//
// Full policy: the producer rejects the NEW element and counts it in dropped(); an ISR
// cannot drop the oldest because the consumer may be reading it.
//
// `depth` (runtime, <= N) lets a config use fewer slots than the compile-time capacity.
template <typename T, std::size_t N>
class IsrRing {
  static_assert(N >= 2 && (N & (N - 1)) == 0, "IsrRing capacity must be a power of two >= 2");
  static_assert(N <= (1u << 30), "IsrRing capacity too large for 32-bit indices");

 public:
  explicit IsrRing(std::size_t depth = N) : depth_(clamp_depth(depth)) {}

  static constexpr std::size_t capacity() { return N; }
  std::size_t depth() const { return depth_; }

  // --- producer side (ISR) ---

  /// Copies `v` in. Returns false (and counts a drop) when full.
  bool push(const T& v) {
    const std::uint32_t h = head_.load(std::memory_order_relaxed);
    const std::uint32_t t = tail_.load(std::memory_order_acquire);
    if (h - t >= depth_) {
      dropped_.store(dropped_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
      return false;
    }
    buf_[h & kMask] = v;
    head_.store(h + 1, std::memory_order_release);
    return true;
  }

  // --- consumer side (main loop) ---

  bool empty() const {
    return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_relaxed);
  }
  std::size_t size() const {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_relaxed);
  }

  /// Oldest element, or nullptr when empty. Valid until pop(); the producer never touches
  /// a slot that has not been popped, so the sink can read it in place without a copy.
  const T* front() const {
    const std::uint32_t t = tail_.load(std::memory_order_relaxed);
    if (head_.load(std::memory_order_acquire) == t) {
      return nullptr;
    }
    return &buf_[t & kMask];
  }

  /// Newest element, or nullptr when empty. Stable: the producer only writes the next slot.
  const T* back() const {
    const std::uint32_t h = head_.load(std::memory_order_acquire);
    if (h == tail_.load(std::memory_order_relaxed)) {
      return nullptr;
    }
    return &buf_[(h - 1) & kMask];
  }

  /// Releases the oldest element. Call only after front() returned non-null.
  void pop() { tail_.store(tail_.load(std::memory_order_relaxed) + 1, std::memory_order_release); }

  /// Elements rejected because the ring was full (wraps at 2^32).
  std::uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  static constexpr std::uint32_t kMask = static_cast<std::uint32_t>(N - 1);
  static std::uint32_t clamp_depth(std::size_t d) {
    return static_cast<std::uint32_t>(d < 1 ? 1 : (d > N ? N : d));
  }

  std::array<T, N> buf_{};
  std::uint32_t depth_;
  std::atomic<std::uint32_t> head_{0};  // next slot to write; written by the producer only
  std::atomic<std::uint32_t> tail_{0};  // next slot to read; written by the consumer only
  std::atomic<std::uint32_t> dropped_{0};
};

}  // namespace mss2d
