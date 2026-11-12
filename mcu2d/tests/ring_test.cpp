#include <atomic>
#include <cstdint>
#include <thread>

#include <gtest/gtest.h>

#include "mss2d/isr_ring.hpp"

namespace mss2d {
namespace {

TEST(IsrRing, EmptyFrontAndBackAreNull) {
  IsrRing<int, 4> r;
  EXPECT_TRUE(r.empty());
  EXPECT_EQ(r.front(), nullptr);
  EXPECT_EQ(r.back(), nullptr);
}

TEST(IsrRing, FifoOrderAndWrap) {
  IsrRing<int, 4> r;
  int next_in = 0;
  int next_out = 0;
  // Many more elements than capacity so the indices wrap several times.
  for (int round = 0; round < 50; ++round) {
    for (int i = 0; i < 3; ++i) {
      ASSERT_TRUE(r.push(next_in++));
    }
    EXPECT_EQ(*r.back(), next_in - 1);
    for (int i = 0; i < 3; ++i) {
      ASSERT_NE(r.front(), nullptr);
      EXPECT_EQ(*r.front(), next_out++);
      r.pop();
    }
  }
  EXPECT_TRUE(r.empty());
  EXPECT_EQ(r.dropped(), 0U);
}

TEST(IsrRing, FullRejectsNewAndCountsDrops) {
  IsrRing<int, 4> r;
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(r.push(i));
  }
  EXPECT_FALSE(r.push(100));
  EXPECT_FALSE(r.push(101));
  EXPECT_EQ(r.dropped(), 2U);
  EXPECT_EQ(r.size(), 4U);
  // The oldest elements survive; the rejected ones are gone.
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(*r.front(), i);
    r.pop();
  }
  EXPECT_TRUE(r.push(7));
  EXPECT_EQ(*r.front(), 7);
}

TEST(IsrRing, RuntimeDepthBelowCapacity) {
  IsrRing<int, 8> r(3);
  EXPECT_TRUE(r.push(1));
  EXPECT_TRUE(r.push(2));
  EXPECT_TRUE(r.push(3));
  EXPECT_FALSE(r.push(4));
  EXPECT_EQ(r.dropped(), 1U);
}

// One thread stands in for the ISR. Run under TSAN in CI-like checks.
TEST(IsrRing, TwoThreadStressKeepsOrderAndAccountsEverything) {
  constexpr std::uint32_t kSent = 300000;
  IsrRing<std::uint32_t, 64> r;
  std::atomic<bool> done{false};

  std::thread producer([&] {
    for (std::uint32_t i = 0; i < kSent; ++i) {
      r.push(i);  // a full ring drops; the sequence number gap shows it
    }
    done.store(true, std::memory_order_release);
  });

  std::uint32_t received = 0;
  bool have_last = false;
  std::uint32_t last = 0;
  bool ordered = true;
  for (;;) {
    const bool finished = done.load(std::memory_order_acquire);
    const std::uint32_t* p = r.front();
    if (p == nullptr) {
      if (finished) {
        break;  // done was read before front(): nothing more can arrive
      }
      continue;
    }
    if (have_last && *p <= last) {
      ordered = false;
    }
    last = *p;
    have_last = true;
    ++received;
    r.pop();
  }
  producer.join();

  EXPECT_TRUE(ordered);
  EXPECT_GT(received, 0U);
  EXPECT_EQ(received + r.dropped(), kSent);
}

}  // namespace
}  // namespace mss2d
