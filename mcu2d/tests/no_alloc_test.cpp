// Proves the library does not touch the heap: this test binary replaces the global operator
// new/delete with counting versions and checks the count around run_once()/flush().
#include <atomic>
#include <cstdlib>
#include <memory>
#include <new>

#include <gtest/gtest.h>

#include "mss2d/merger.hpp"

namespace {
std::atomic<long> g_news{0};
}

void* operator new(std::size_t n) {
  g_news.fetch_add(1, std::memory_order_relaxed);
  if (void* p = std::malloc(n)) {
    return p;
  }
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}

namespace mss2d {
namespace {

struct Counting : Sink {
  int n = 0;
  void on_imu(const ImuSample2d&) override { ++n; }
  void on_wheel(const WheelSample&) override { ++n; }
  void on_scan(const Scan2d&) override { ++n; }
};

TEST(NoAlloc, PushRunOnceAndFlushDoNotAllocate) {
  auto m = std::make_unique<Merger>(Config{});
  auto scan = std::make_unique<Scan2d>();
  scan->count = 2;
  scan->points[1].t_offset_ns = 100;
  Counting sink;

  const long before = g_news.load();
  for (int i = 1; i <= 300; ++i) {
    ImuSample2d s;
    s.t = Timestamp{i * 5000000LL};
    (void)m->push_imu(s);
    if (i % 20 == 0) {
      scan->t_start = Timestamp{i * 5000000LL - 50};
      (void)m->push_scan(*scan);
    }
    m->run_once(sink);
  }
  m->flush(sink);
  const long after = g_news.load();

  EXPECT_GT(sink.n, 300);
  EXPECT_EQ(after - before, 0);
}

}  // namespace
}  // namespace mss2d
