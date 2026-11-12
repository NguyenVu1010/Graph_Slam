// Lock-free primitives of the estimator threads (ADR-033): SpscQueue and SnapshotSlots.
// The two-thread tests are meant to run under ThreadSanitizer too (tools/wsl/build.sh --tsan,
// CI job tsan); they also check the data, so a plain build catches torn reads.
#include <array>
#include <atomic>
#include <cstdint>
#include <thread>

#include <gtest/gtest.h>

#include "mss/pipeline/snapshot_slots.hpp"
#include "mss/pipeline/spsc_queue.hpp"

#include "alloc_counter.hpp"

namespace {

using mss::pipeline::SnapshotSlots;
using mss::pipeline::SpscQueue;

TEST(SpscQueue, FifoAndRejectsWhenFull) {
  SpscQueue<int, 3> q;
  EXPECT_EQ(q.capacity(), 3U);
  EXPECT_TRUE(q.empty());
  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));
  EXPECT_TRUE(q.push(3));
  EXPECT_FALSE(q.push(4));  // full: the new element is rejected, the queue is unchanged
  EXPECT_EQ(q.size(), 3U);
  int v = 0;
  ASSERT_TRUE(q.pop(v));
  EXPECT_EQ(v, 1);
  EXPECT_TRUE(q.push(5));  // room again, wraps around
  ASSERT_TRUE(q.pop(v));
  EXPECT_EQ(v, 2);
  ASSERT_TRUE(q.pop(v));
  EXPECT_EQ(v, 3);
  ASSERT_TRUE(q.pop(v));
  EXPECT_EQ(v, 5);
  EXPECT_FALSE(q.pop(v));
  EXPECT_EQ(v, 5);  // unchanged on empty
  EXPECT_TRUE(q.empty());
}

TEST(SpscQueue, PushPopDoNotAllocate) {
  SpscQueue<std::array<double, 8>, 16> q;
  std::array<double, 8> in{};
  std::array<double, 8> out{};
  AllocCounter::reset();
  for (int i = 0; i < 100; ++i) {
    in[0] = i;
    ASSERT_TRUE(q.push(in));
    ASSERT_TRUE(q.pop(out));
  }
  EXPECT_EQ(AllocCounter::count_this_thread(), 0U);
}

// One producer, one consumer: every value is received once, in order, or counted as rejected.
TEST(SpscQueue, TwoThreadsKeepOrderAndCountRejects) {
  constexpr std::int64_t kN = 200000;
  SpscQueue<std::int64_t, 64> q;
  std::int64_t rejected = 0;
  std::thread producer([&] {
    for (std::int64_t i = 0; i < kN; ++i) {
      if (!q.push(i)) {
        ++rejected;
      }
    }
    while (!q.push(-1)) {  // end marker, never dropped
      std::this_thread::yield();
    }
  });
  std::int64_t received = 0;
  std::int64_t last = -1;
  bool ordered = true;
  for (;;) {
    std::int64_t v = 0;
    if (!q.pop(v)) {
      std::this_thread::yield();
      continue;
    }
    if (v < 0) {
      break;
    }
    ordered = ordered && v > last;
    last = v;
    ++received;
  }
  producer.join();
  EXPECT_TRUE(ordered);
  EXPECT_EQ(received + rejected, kN);
}

using Buf = std::array<std::int64_t, 256>;

TEST(SnapshotSlots, ReaderSeesLatestPublished) {
  SnapshotSlots<Buf, 3> s;
  {
    const auto r = s.acquire();
    EXPECT_FALSE(r.has_value());  // nothing published yet
  }
  Buf* w = s.begin_write();
  ASSERT_NE(w, nullptr);
  w->fill(7);
  s.publish();
  {
    const auto r = s.acquire();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->front(), 7);
    EXPECT_EQ(r.version(), 1U);
    // While the reader holds version 1, the writer still finds a free slot.
    Buf* w2 = s.begin_write();
    ASSERT_NE(w2, nullptr);
    EXPECT_NE(w2, w);
    w2->fill(8);
    s.publish();
    EXPECT_EQ(r->back(), 7);  // the held snapshot is not touched
  }
  const auto r = s.acquire();
  EXPECT_EQ(r->front(), 8);
  EXPECT_EQ(r.version(), 2U);
}

TEST(SnapshotSlots, AcquireDoesNotAllocate) {
  SnapshotSlots<Buf, 3> s;
  s.begin_write()->fill(1);
  s.publish();
  AllocCounter::reset();
  for (int i = 0; i < 100; ++i) {
    const auto r = s.acquire();
    ASSERT_TRUE(r.has_value());
  }
  EXPECT_EQ(AllocCounter::count_this_thread(), 0U);
}

// The writer fills a whole buffer with its version; a reader must never see a mixed buffer.
TEST(SnapshotSlots, TwoThreadsNeverTear) {
  constexpr std::int64_t kVersions = 20000;
  SnapshotSlots<Buf, 3> s;
  std::atomic<std::int64_t> no_slot{0};
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (std::int64_t v = 1; v <= kVersions; ++v) {
      Buf* w = s.begin_write();
      if (w == nullptr) {
        no_slot.fetch_add(1);
        continue;
      }
      w->fill(v);
      s.publish();
    }
    done.store(true);
  });
  bool consistent = true;
  bool monotonic = true;
  std::int64_t last = 0;
  while (!done.load()) {
    const auto r = s.acquire();
    if (!r.has_value()) {
      continue;
    }
    const std::int64_t first = r->front();
    for (const std::int64_t x : *r) {
      consistent = consistent && x == first;
    }
    monotonic = monotonic && first >= last;
    last = first;
  }
  writer.join();
  EXPECT_TRUE(consistent);
  EXPECT_TRUE(monotonic);
  EXPECT_EQ(s.acquire()->front(), kVersions);
  EXPECT_EQ(no_slot.load(), 0);  // one reader holding one slot always leaves the writer a free one
}

}  // namespace
