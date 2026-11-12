#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace mss::pipeline {

/// Immutable snapshots of a large value (the front-end map, ADR-033 decision 3) handed from
/// one writer thread to reader threads without locks and without allocation.
///
/// N buffers live inline. The writer fills a free buffer (begin_write(), then publish());
/// a reader acquire()s the latest published buffer and holds it, unchanged, until its
/// Ref goes away. A buffer is free when it is neither the published one nor held by a
/// reader, so with one reader holding at most one Ref, N = 3 always leaves the writer a
/// free buffer. Readers never wait on the writer; the writer never waits on readers
/// (begin_write() returns nullptr instead).
///
/// Correctness: a reader announces itself on a buffer (readers_ count) and then re-checks
/// that the buffer is still the published one before touching it; the writer checks the
/// count after the buffer stopped being published. Both use sequentially consistent
/// atomics, so one of them always sees the other.
///
/// Threading: begin_write() and publish() from one writer thread only; acquire() from any
/// thread. T must be default-constructible. Store the whole object where it lives for the
/// program's lifetime (it may be large; allocate it once at start-up).
template <class T, std::size_t N>
class SnapshotSlots {
  static_assert(N >= 2, "SnapshotSlots needs at least two buffers");

 public:
  /// A held snapshot. Move-only; releases the buffer on destruction.
  class Ref {
   public:
    Ref() = default;
    Ref(const Ref&) = delete;
    Ref& operator=(const Ref&) = delete;
    Ref(Ref&& o) noexcept : owner_(o.owner_), slot_(o.slot_) { o.owner_ = nullptr; }
    Ref& operator=(Ref&& o) noexcept {
      if (this != &o) {
        release();
        owner_ = o.owner_;
        slot_ = o.slot_;
        o.owner_ = nullptr;
      }
      return *this;
    }
    ~Ref() { release(); }

    bool has_value() const { return owner_ != nullptr; }
    /// Precondition for the accessors: has_value().
    const T& operator*() const { return owner_->slots_[slot_]; }
    const T* operator->() const { return &owner_->slots_[slot_]; }
    /// 1 for the first publish(), +1 for each later one.
    std::uint64_t version() const { return owner_->versions_[slot_]; }

   private:
    friend class SnapshotSlots;
    Ref(const SnapshotSlots* owner, std::size_t slot) : owner_(owner), slot_(slot) {}
    void release() {
      if (owner_ != nullptr) {
        owner_->readers_[slot_].fetch_sub(1);
        owner_ = nullptr;
      }
    }

    const SnapshotSlots* owner_ = nullptr;
    std::size_t slot_ = 0;
  };

  /// Writer only. A free buffer to fill (its old contents are unspecified), or nullptr when
  /// every buffer is published or held. Calling it again before publish() returns a buffer
  /// again and abandons the previous one.
  T* begin_write() {
    const int published = published_.load();
    for (std::size_t i = 0; i < N; ++i) {
      if (static_cast<int>(i) != published && readers_[i].load() == 0) {
        writing_ = static_cast<int>(i);
        return &slots_[i];
      }
    }
    writing_ = -1;
    return nullptr;
  }

  /// Writer only. Makes the buffer from the last successful begin_write() the latest
  /// snapshot. Precondition: that begin_write() returned non-null and was not published yet.
  void publish() {
    if (writing_ < 0) {
      return;
    }
    versions_[static_cast<std::size_t>(writing_)] = ++next_version_;
    published_.store(writing_);
    writing_ = -1;
  }

  /// Any thread. The latest published snapshot, or an empty Ref before the first publish().
  Ref acquire() const {
    for (;;) {
      const int i = published_.load();
      if (i < 0) {
        return Ref{};
      }
      const auto slot = static_cast<std::size_t>(i);
      readers_[slot].fetch_add(1);
      if (published_.load() == i) {
        return Ref{this, slot};
      }
      readers_[slot].fetch_sub(1);  // republished meanwhile: retry on the new one
    }
  }

 private:
  std::array<T, N> slots_{};
  std::array<std::uint64_t, N> versions_{};
  mutable std::array<std::atomic<std::uint32_t>, N> readers_{};
  std::atomic<int> published_{-1};
  int writing_ = -1;                // writer thread only
  std::uint64_t next_version_ = 0;  // writer thread only
};

}  // namespace mss::pipeline
