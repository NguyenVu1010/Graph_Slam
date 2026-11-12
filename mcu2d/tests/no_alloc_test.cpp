// Proves the library does not touch the heap: this test binary replaces the global operator
// new/delete with counting versions and checks the count around run_once()/flush().
#include <atomic>
#include <cstdlib>
#include <memory>
#include <new>

#include <gtest/gtest.h>

#include "mss2d/merger.hpp"
#include "mss2d/scan_matcher.hpp"
#include "sim_world.hpp"

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

TEST(NoAlloc, GridInsertDistanceFieldAndMatchDoNotAllocate) {
  const Config cfg;
  auto grid = std::make_unique<OccupancyGrid>(cfg.grid);
  auto df = std::make_unique<DistanceField>(cfg.grid);
  auto matcher = std::make_unique<ScanMatcher>(cfg.matcher);
  auto scan = std::make_unique<Scan2d>();
  auto pts = std::make_unique<PointCloud2d>();
  const sim::World world = sim::room_world();
  sim::make_scan(world, Pose2T<double>(0.2, -0.1, 0.3), 720, 12.0, 0.0, 1, *scan);

  const long before = g_news.load();
  grid->reset(Pose2());
  scan_to_points(*scan, cfg.matcher.min_range_m, cfg.matcher.max_range_m, *pts);
  grid->insert(*pts, Pose2(0.2F, -0.1F, 0.3F));
  df->compute(*grid);
  const MatchResult r = matcher->match(*grid, *df, *pts, Pose2(0.3F, -0.2F, 0.35F));
  const long after = g_news.load();

  EXPECT_EQ(r.status, MatchStatus::kOk);
  EXPECT_EQ(after - before, 0);
}

}  // namespace
}  // namespace mss2d
