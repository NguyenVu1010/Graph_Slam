#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mss/preprocess/detail/voxel_set.hpp"

#include "alloc_counter.hpp"

namespace mss::preprocess::detail {
namespace {

TEST(VoxelKey, FloorsNegativeCoordinates) {
  EXPECT_EQ(voxel_key(-0.1F, 0.1F, -0.5F, 0.5), (VoxelKey{-1, 0, -1}));
  EXPECT_EQ(voxel_key(0.5F, -0.0F, 0.49F, 0.5), (VoxelKey{1, 0, 0}));
}

TEST(VoxelSet, InsertReportsNewOnlyOnce) {
  VoxelSet set(10);
  EXPECT_TRUE(set.insert(VoxelKey{1, -2, 3}));
  EXPECT_FALSE(set.insert(VoxelKey{1, -2, 3}));
  EXPECT_EQ(set.size(), 1U);
}

TEST(VoxelSet, FillsToCapacityWithDistinctKeys) {
  VoxelSet set(1000);
  for (std::int64_t x = -5; x < 5; ++x) {
    for (std::int64_t y = -5; y < 5; ++y) {
      for (std::int64_t z = -5; z < 5; ++z) {
        EXPECT_TRUE(set.insert(VoxelKey{x, y, z}));
      }
    }
  }
  EXPECT_EQ(set.size(), 1000U);
}

TEST(VoxelSet, ClearForgetsAllKeys) {
  VoxelSet set(10);
  ASSERT_TRUE(set.insert(VoxelKey{0, 0, 0}));
  ASSERT_TRUE(set.insert(VoxelKey{-1, 7, 2}));
  set.clear();
  EXPECT_EQ(set.size(), 0U);
  EXPECT_TRUE(set.insert(VoxelKey{0, 0, 0}));
  EXPECT_TRUE(set.insert(VoxelKey{-1, 7, 2}));
}

TEST(VoxelSet, ZeroCapacityThrows) {
  EXPECT_THROW(VoxelSet(0), std::invalid_argument);
}

TEST(VoxelSet, InsertAndClearDoNotAllocate) {
  VoxelSet set(1000);
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    set.clear();
    for (std::int64_t i = 0; i < 1000; ++i) {
      set.insert(VoxelKey{i, -i, i / 3});
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_EQ(set.size(), 1000U);
}

}  // namespace
}  // namespace mss::preprocess::detail
