#include <array>
#include <cstdint>
#include <new>
#include <type_traits>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/span.hpp"

#include "alloc_counter.hpp"

namespace mss::common {

namespace {
void fill(ScanFrame& f, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    const auto k = static_cast<float>(i);
    ASSERT_TRUE(f.push(k,
                       k + 0.5F,
                       k + 1.0F,
                       2.0F * k,
                       static_cast<std::uint32_t>(i * 10),
                       static_cast<std::uint16_t>(i % 32)));
  }
}
}  // namespace

TEST(ScanFrame, ReserveOnceThenPushNoAlloc) {
  ScanFrame f;
  f.reserve(1000);
  AllocCounter::reset();
  std::size_t allocs = 0;
  {
    const EigenMallocGuard no_eigen_malloc;
    fill(f, 1000);
    allocs = AllocCounter::count();
  }
  EXPECT_EQ(allocs, 0U);
  EXPECT_EQ(f.size(), 1000U);
  EXPECT_EQ(f.xyz().size(), 3000U);
  EXPECT_FLOAT_EQ(f.xyz()[3 * 7 + 1], 7.5F);
  EXPECT_EQ(f.ring()[33], 1U);
}

TEST(ScanFrame, PushWhenFullReturnsFalse) {
  ScanFrame f(1000);
  fill(f, 1000);
  AllocCounter::reset();
  const bool pushed = f.push(0, 0, 0, 0, 0, 0);
  const std::size_t allocs = AllocCounter::count();
  EXPECT_FALSE(pushed);
  EXPECT_EQ(allocs, 0U);
  EXPECT_EQ(f.size(), 1000U);
}

TEST(ScanFrame, CopyFromNoAlloc) {
  ScanFrame a(1000);
  a.t_start = Timestamp{12345};
  fill(a, 500);
  ScanFrame b(1000);
  AllocCounter::reset();
  EXPECT_TRUE(b.copy_from(a));
  EXPECT_EQ(AllocCounter::count(), 0U);
  ASSERT_EQ(b.size(), 500U);
  EXPECT_EQ(b.t_start, a.t_start);
  for (std::size_t i = 0; i < 500; ++i) {
    EXPECT_EQ(b.xyz()[3 * i], a.xyz()[3 * i]);
    EXPECT_EQ(b.xyz()[3 * i + 2], a.xyz()[3 * i + 2]);
    EXPECT_EQ(b.intensity()[i], a.intensity()[i]);
    EXPECT_EQ(b.t_offset_ns()[i], a.t_offset_ns()[i]);
    EXPECT_EQ(b.ring()[i], a.ring()[i]);
  }
  ScanFrame small(10);
  EXPECT_FALSE(small.copy_from(a));
  EXPECT_EQ(small.size(), 0U);
}

TEST(ScanFrame, MoveKeepsData) {
  ScanFrame a(100);
  fill(a, 50);
  ScanFrame b(std::move(a));
  EXPECT_EQ(b.size(), 50U);
  EXPECT_EQ(b.capacity(), 100U);
  EXPECT_FLOAT_EQ(b.intensity()[10], 20.0F);
  ScanFrame c;
  c = std::move(b);
  EXPECT_EQ(c.size(), 50U);
  EXPECT_FLOAT_EQ(c.xyz()[3 * 49], 49.0F);
}

TEST(ScanFrame, MovedFromIsEmptyAndReusable) {
  ScanFrame a(100);
  fill(a, 50);
  ScanFrame b(std::move(a));
  EXPECT_EQ(a.size(), 0U);
  EXPECT_EQ(a.capacity(), 0U);
  EXPECT_TRUE(a.xyz().empty());
  EXPECT_FALSE(a.push(1, 2, 3, 4, 5, 6));
  a.reserve(10);
  EXPECT_EQ(a.capacity(), 10U);
  EXPECT_TRUE(a.push(1, 2, 3, 4, 5, 6));
  ScanFrame c(20);
  fill(c, 5);
  c = std::move(b);
  EXPECT_EQ(c.size(), 50U);
  EXPECT_EQ(b.capacity(), 0U);
  EXPECT_TRUE(c.copy_from(c));
  EXPECT_EQ(c.size(), 50U);
}

TEST(AllocCounter, CountsOneHeapAllocation) {
  AllocCounter::reset();
  int* volatile p = new int(7);
  EXPECT_EQ(AllocCounter::count(), 1U);
  delete p;
}

// Over-aligned types go through operator new(std::size_t, std::align_val_t).
struct alignas(64) OverAligned {
  double v[8];
};

TEST(AllocCounter, CountsAlignedAndNothrowAllocations) {
  AllocCounter::reset();
  OverAligned* volatile a = new OverAligned();
  const std::size_t after_aligned = AllocCounter::count();
  int* volatile b = new (std::nothrow) int(3);
  const std::size_t after_nothrow = AllocCounter::count();
  EXPECT_EQ(after_aligned, 1U);
  EXPECT_EQ(after_nothrow, 2U);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(a) % 64U, 0U);
  delete a;
  delete b;
}

// Positive control for EigenMallocGuard: a dynamic Eigen allocation under the guard must
// abort (EIGEN_RUNTIME_NO_MALLOC + live eigen_assert, see tests/CMakeLists.txt).
TEST(EigenMallocGuardDeathTest, DynamicEigenAllocationAborts) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(
      {
        const EigenMallocGuard no_eigen_malloc;
        Eigen::VectorXd v(64);
        v.setOnes();
        const volatile double sum = v.sum();  // keep the allocation observable
        (void)sum;
      },
      "heap allocation is forbidden");
  Eigen::VectorXd ok(64);  // outside the guard Eigen may allocate again
  ok.setOnes();
  EXPECT_DOUBLE_EQ(ok.sum(), 64.0);
}

TEST(ScanFrame, ClearKeepsCapacity) {
  ScanFrame f(100);
  fill(f, 40);
  f.clear();
  EXPECT_EQ(f.size(), 0U);
  EXPECT_EQ(f.capacity(), 100U);
  EXPECT_TRUE(f.xyz().empty());
}

TEST(ScanFrame, BulkWriteThenSetSize) {
  ScanFrame f(8);
  f.xyz_mut()[0] = 1.0F;
  f.xyz_mut()[1] = 2.0F;
  f.xyz_mut()[2] = 3.0F;
  f.intensity_mut()[0] = 9.0F;
  f.t_offset_ns_mut()[0] = 77;
  f.ring_mut()[0] = 5;
  EXPECT_FALSE(f.set_size(9));
  EXPECT_EQ(f.size(), 0U);
  EXPECT_TRUE(f.set_size(1));
  EXPECT_FLOAT_EQ(f.xyz()[2], 3.0F);
  EXPECT_EQ(f.t_offset_ns()[0], 77U);
  EXPECT_EQ(f.ring()[0], 5U);
}

TEST(ScanFrame, ReserveAgainOnlyWhenEmpty) {
  ScanFrame f(10);
  fill(f, 3);
  f.reserve(20);  // ignored: not empty
  EXPECT_EQ(f.capacity(), 10U);
  f.clear();
  f.reserve(20);
  EXPECT_EQ(f.capacity(), 20U);
}

TEST(Span, ConstFromVectorAndArray) {
  const std::vector<int> v{1, 2, 3};
  const Span<const int> sv = v;
  EXPECT_EQ(sv.size(), 3U);
  EXPECT_EQ(sv[2], 3);
  const std::array<int, 2> a{4, 5};
  const Span<const int> sa = a;
  EXPECT_EQ(sa.size(), 2U);
  int sum = 0;
  for (int x : sa) {
    sum += x;
  }
  EXPECT_EQ(sum, 9);
  EXPECT_TRUE(Span<const int>().empty());
}

TEST(Span, MutableToConstAndNoTemporaries) {
  std::vector<int> v{1, 2, 3};
  const Span<int> mut(v.data(), v.size());
  const Span<const int> view = mut;
  EXPECT_EQ(view.size(), 3U);
  EXPECT_EQ(view.data(), v.data());
  static_assert(std::is_convertible_v<Span<int>, Span<const int>>);
  static_assert(!std::is_convertible_v<Span<const int>, Span<int>>);
  static_assert(!std::is_constructible_v<Span<const int>, std::vector<int>&&>);
  static_assert(!std::is_constructible_v<Span<const int>, std::array<int, 2>&&>);
  static_assert(std::is_constructible_v<Span<const int>, const std::vector<int>&>);
  static_assert(std::is_constructible_v<Span<const int>, std::vector<int>&>);
}

TEST(ScanFrame, EndTimeIsStartPlusMaxOffset) {
  ScanFrame f(4);
  f.t_start = Timestamp{1000};
  EXPECT_EQ(scan_end_time(f), Timestamp{1000});  // no points: t_start
  ASSERT_TRUE(f.push(0.0F, 0.0F, 0.0F, 0.0F, 50U, 0));
  ASSERT_TRUE(f.push(0.0F, 0.0F, 0.0F, 0.0F, 700U, 0));
  ASSERT_TRUE(f.push(0.0F, 0.0F, 0.0F, 0.0F, 20U, 0));  // offsets need not be sorted
  EXPECT_EQ(scan_end_time(f), Timestamp{1700});
}

}  // namespace mss::common
