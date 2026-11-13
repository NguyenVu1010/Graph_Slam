// Unit tests of the front-end map snapshots (ADR-033 decision 3): MapPublisher, the writer the
// keyframe backend updates the VoxelMap through, and SnapshotSlots' in-place constructor.
// Single-threaded: the reader patterns are simulated by holding Refs.
#include <cstddef>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/map/config.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/pipeline/snapshot_slots.hpp"

#include "alloc_counter.hpp"
#include "keyframe_store.hpp"
#include "map_publisher.hpp"

namespace mss::pipeline::detail {
namespace {

using common::KeyId;
using common::Pose;
using common::ScanFrame;
using common::Vec3;

constexpr std::size_t kWindow = 4;  // map.window_keyframes of the tests
constexpr std::size_t kCloudPoints = 200;

map::MapConfig small_map_config() {
  map::MapConfig c;
  c.voxel_m = 0.5;
  c.max_points_per_voxel = 4;
  c.min_point_dist_m = 0.05;  // spacing on: num_spacing_replaced() must match too
  c.knn_k = 5;
  c.prune_radius_m = 3.0;
  c.prune_period_s = 1.0;
  c.max_voxels = 300;  // small: some inserts drop points (num_dropped)
  return c;
}

ScanFrame random_cloud(std::mt19937& rng) {
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  ScanFrame s(kCloudPoints);
  for (std::size_t i = 0; i < kCloudPoints; ++i) {
    s.push(static_cast<float>(u(rng)),
           static_cast<float>(u(rng)),
           static_cast<float>(0.3 * u(rng)),
           0.0F,
           0U,
           0U);
  }
  return s;
}

Pose shifted(double x_m, double y_m) {
  Pose T;
  T.p = Vec3(x_m, y_m, 0.0);
  return T;
}

// The two maps answer every query identically and have the same counters.
void expect_same_map(const map::VoxelMap& a, const map::VoxelMap& b) {
  EXPECT_EQ(a.num_voxels(), b.num_voxels());
  EXPECT_EQ(a.num_points(), b.num_points());
  EXPECT_EQ(a.num_dropped(), b.num_dropped());
  EXPECT_EQ(a.num_spacing_replaced(), b.num_spacing_replaced());
  map::NeighborBuffer na;
  map::NeighborBuffer nb;
  for (double x = -6.0; x <= 6.0; x += 0.7) {
    for (double y = -6.0; y <= 6.0; y += 0.7) {
      const Vec3 q(x, y, 0.1);
      ASSERT_EQ(a.knn(q, 0, na), b.knn(q, 0, nb));
      for (int k = 0; k < na.size; ++k) {
        const auto u = static_cast<std::size_t>(k);
        ASSERT_EQ(na.dist2[u], nb.dist2[u]);
        ASSERT_EQ(na.p_w[u], nb.p_w[u]);
      }
    }
  }
}

// A publisher and the single map of the old pipeline, given the same operations.
struct Harness {
  explicit Harness(std::size_t max_keyframes = 64)
      : cfg(small_map_config()), store(max_keyframes), pub(cfg, store, kWindow), ref(cfg) {}

  void add_keyframe(KeyId id, const Pose& T_w_l) {
    const ScanFrame cloud = random_cloud(rng);
    ASSERT_TRUE(store.add(id, common::Timestamp{static_cast<std::int64_t>(id)}, cloud).is_ok());
    ref.insert(cloud, T_w_l);
    ASSERT_TRUE(pub.begin());
    pub.insert(id, cloud, T_w_l);
    pub.publish();
  }
  void prune(const Vec3& p) {
    ref.prune(p);
    ASSERT_TRUE(pub.begin());
    pub.prune(p);
    pub.publish();
  }
  // Rebuild of the last `n` keyframes, each shifted by dx.
  void rebuild(KeyId last, std::size_t n, double dx) {
    ref.clear();
    ASSERT_TRUE(pub.begin());
    pub.clear();
    for (KeyId id = last + 1 - static_cast<KeyId>(n); id <= last; ++id) {
      const KeyframeEntry* e = store.find(id);
      ASSERT_NE(e, nullptr);
      const Pose T = shifted(0.1 * static_cast<double>(id) + dx, 0.0);
      ref.insert(e->cloud_l, T);
      pub.insert(id, e->cloud_l, T);
    }
    pub.publish();
  }

  map::MapConfig cfg;
  KeyframeStore store;
  MapPublisher pub;
  map::VoxelMap ref;
  std::mt19937 rng{7};
};

TEST(MapPublisher, PublishesTheEmptyMapFirst) {
  Harness h;
  const MapRef r = h.pub.acquire();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r.version(), 1U);
  EXPECT_EQ(r->map.num_points(), 0U);
}

TEST(MapPublisher, HeldSnapshotIsNotChangedByAPublish) {
  Harness h;
  const MapRef before = h.pub.acquire();
  ASSERT_NO_FATAL_FAILURE(h.add_keyframe(0, shifted(0.0, 0.0)));
  // No stale read after the publish: a new acquire() sees the insert...
  const MapRef after = h.pub.acquire();
  EXPECT_EQ(after.version(), 2U);
  EXPECT_GT(after->map.num_points(), 0U);
  expect_same_map(after->map, h.ref);
  // ...and the snapshot held across it is unchanged.
  EXPECT_EQ(before.version(), 1U);
  EXPECT_EQ(before->map.num_points(), 0U);
}

TEST(MapPublisher, RebuildPublishesANewVersion) {
  Harness h;
  for (KeyId id = 0; id < 6; ++id) {
    ASSERT_NO_FATAL_FAILURE(h.add_keyframe(id, shifted(0.1 * static_cast<double>(id), 0.0)));
  }
  const std::uint64_t v = h.pub.acquire().version();
  ASSERT_NO_FATAL_FAILURE(h.rebuild(5, kWindow, 0.3));
  const MapRef r = h.pub.acquire();
  EXPECT_EQ(r.version(), v + 1);
  expect_same_map(r->map, h.ref);
}

// Bit-identical to one map: buffers that lag by one or two updates (the reader holding the
// snapshot across the backend's updates, as the front-end does for a scan) replay the log.
TEST(MapPublisher, EveryPublishedMapEqualsTheSingleMap) {
  Harness h;
  std::mt19937 pick(3);
  KeyId next = 0;
  for (int scan = 0; scan < 120; ++scan) {
    const MapRef held = h.pub.acquire();  // the front-end's snapshot for this scan
    const int kind = static_cast<int>(pick() % 6U);
    if (kind <= 2) {
      ASSERT_NO_FATAL_FAILURE(h.add_keyframe(next, shifted(0.05 * scan, 0.02 * scan)));
      ++next;
    }
    if (kind == 3) {
      ASSERT_NO_FATAL_FAILURE(h.prune(Vec3(0.05 * scan, 0.0, 0.0)));
    }
    if (kind == 4 && next >= kWindow) {
      ASSERT_NO_FATAL_FAILURE(h.rebuild(next - 1, kWindow, 0.01 * scan));
    }
    const MapRef now = h.pub.acquire();
    ASSERT_NO_FATAL_FAILURE(expect_same_map(now->map, h.ref)) << "scan " << scan;
  }
  EXPECT_EQ(h.pub.n_full_copies(), 0U);
  EXPECT_EQ(h.pub.n_publish_skipped(), 0U);
}

// A buffer held for longer than the log covers is copied from the published one instead.
TEST(MapPublisher, LongHeldBufferIsCopied) {
  Harness h;
  MapRef old = h.pub.acquire();  // the first buffer, version 1
  const std::size_t n = kWindow + kMapLogExtraOps + 5;
  for (std::size_t k = 0; k < n; ++k) {
    ASSERT_NO_FATAL_FAILURE(h.prune(Vec3(0.0, 0.0, 0.0)));
  }
  ASSERT_NO_FATAL_FAILURE(h.add_keyframe(0, shifted(0.0, 0.0)));
  EXPECT_EQ(h.pub.n_full_copies(), 0U);  // two buffers alternated
  old = MapRef{};
  ASSERT_NO_FATAL_FAILURE(h.add_keyframe(1, shifted(0.2, 0.0)));  // takes the first buffer
  EXPECT_EQ(h.pub.n_full_copies(), 1U);
  expect_same_map(h.pub.acquire()->map, h.ref);
}

// A logged insert whose cloud the store evicted cannot be replayed: copied instead.
TEST(MapPublisher, EvictedCloudIsCopied) {
  Harness h(/*max_keyframes=*/1);
  ASSERT_NO_FATAL_FAILURE(h.add_keyframe(0, shifted(0.0, 0.0)));
  EXPECT_EQ(h.pub.n_full_copies(), 0U);
  // The store evicts keyframe 0 before the update; the free buffer still lacks its insert.
  ASSERT_NO_FATAL_FAILURE(h.add_keyframe(1, shifted(0.3, 0.0)));
  EXPECT_EQ(h.pub.n_full_copies(), 1U);
  expect_same_map(h.pub.acquire()->map, h.ref);
}

TEST(MapPublisher, NoFreeBufferDropsTheUpdate) {
  Harness h;
  const MapRef a = h.pub.acquire();
  ASSERT_NO_FATAL_FAILURE(h.add_keyframe(0, shifted(0.0, 0.0)));
  const MapRef b = h.pub.acquire();
  ASSERT_NO_FATAL_FAILURE(h.add_keyframe(1, shifted(0.1, 0.0)));
  const MapRef c = h.pub.acquire();  // every buffer held
  const std::uint64_t v = c.version();
  EXPECT_FALSE(h.pub.begin());
  h.pub.prune(Vec3::Zero());  // no-ops
  h.pub.clear();
  h.pub.publish();
  EXPECT_EQ(h.pub.acquire().version(), v);
  EXPECT_EQ(h.pub.n_publish_skipped(), 1U);
  EXPECT_TRUE(h.pub.lost_update());
  EXPECT_EQ(h.pub.map().num_points(), c->map.num_points());
}

TEST(MapPublisher, LostUpdateClearedByAClear) {
  Harness h;
  {
    const MapRef a = h.pub.acquire();
    ASSERT_NO_FATAL_FAILURE(h.add_keyframe(0, shifted(0.0, 0.0)));
    const MapRef b = h.pub.acquire();
    ASSERT_NO_FATAL_FAILURE(h.add_keyframe(1, shifted(0.1, 0.0)));
    const MapRef c = h.pub.acquire();
    EXPECT_FALSE(h.pub.begin());
    h.pub.publish();
  }
  ASSERT_TRUE(h.pub.begin());
  EXPECT_TRUE(h.pub.lost_update());
  h.pub.clear();
  EXPECT_FALSE(h.pub.lost_update());
  h.pub.publish();
  EXPECT_EQ(h.pub.acquire()->map.num_points(), 0U);
}

TEST(MapPublisher, SteadyStateDoesNotAllocate) {
  Harness h;
  for (KeyId id = 0; id < 8; ++id) {
    ASSERT_NO_FATAL_FAILURE(h.add_keyframe(id, shifted(0.1 * static_cast<double>(id), 0.0)));
  }
  const ScanFrame& cloud = h.store.find(7)->cloud_l;
  map::VoxelMap copy(h.cfg);
  AllocCounter::reset();
  std::size_t allocs = 0;
  {
    const EigenMallocGuard no_eigen_malloc;
    for (int k = 0; k < 20; ++k) {
      const MapRef held = h.pub.acquire();
      (void)h.pub.begin();
      h.pub.insert(7, cloud, shifted(0.01 * k, 0.0));
      h.pub.prune(Vec3::Zero());
      h.pub.publish();
    }
    copy = h.pub.acquire()->map;  // VoxelMap copy assignment, same config
    allocs = AllocCounter::count_this_thread();
  }
  EXPECT_EQ(allocs, 0U);
  EXPECT_EQ(copy.num_points(), h.pub.acquire()->map.num_points());
}

struct NoDefault {
  explicit NoDefault(int v) : value(v) {}
  int value;
};

TEST(SnapshotSlots, InPlaceConstructsEveryBuffer) {
  using Slots = SnapshotSlots<NoDefault, 3>;
  Slots s(std::in_place, 5);
  EXPECT_FALSE(s.acquire().has_value());  // nothing published
  std::vector<Slots::Ref> held;           // holding each one makes the writer take the next
  held.reserve(3);
  for (int k = 0; k < 3; ++k) {
    NoDefault* w = s.begin_write();
    ASSERT_NE(w, nullptr);
    EXPECT_EQ(w->value, 5);
    w->value = 10 + k;
    s.publish();
    held.push_back(s.acquire());
  }
  EXPECT_EQ(s.acquire()->value, 12);
}

}  // namespace
}  // namespace mss::pipeline::detail
