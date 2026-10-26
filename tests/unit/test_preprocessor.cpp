#include <cstddef>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mss/common/types.hpp"
#include "mss/preprocess/config.hpp"
#include "mss/preprocess/preprocessor.hpp"

#include "alloc_counter.hpp"

namespace mss::preprocess {
namespace {

// config/default.yaml preprocess.* and realtime.max_points_per_scan.
constexpr std::size_t kMaxPointsPerScan = 65536;
const PreprocessConfig kDefaultCfg{0.5, 100.0, 0.5, true};

Preprocessor make(const PreprocessConfig& cfg, std::size_t max_points) {
  return Preprocessor(cfg, common::Pose{}, max_points);
}

TEST(Preprocessor, ConstructsWithDefaultYamlValues) {
  EXPECT_NO_THROW(make(kDefaultCfg, kMaxPointsPerScan));
}

TEST(Preprocessor, VoxelZeroIsValid) {
  EXPECT_NO_THROW(make({0.5, 100.0, 0.0, true}, 10));
}

TEST(Preprocessor, RejectsInvalidConfig) {
  EXPECT_THROW(make({-0.1, 100.0, 0.5, true}, 10), std::invalid_argument);  // min < 0
  EXPECT_THROW(make({0.5, 0.5, 0.5, true}, 10), std::invalid_argument);     // max <= min
  EXPECT_THROW(make({0.5, 0.4, 0.5, true}, 10), std::invalid_argument);
  EXPECT_THROW(make({0.5, 100.0, -0.5, true}, 10), std::invalid_argument);  // voxel < 0
  EXPECT_THROW(make({0.5, 100.0, 0.5, true}, 0), std::invalid_argument);    // max_points == 0
}

TEST(Preprocessor, RejectsNonFiniteConfig) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {nan, inf}) {
    EXPECT_THROW(make({bad, 100.0, 0.5, true}, 10), std::invalid_argument);
    EXPECT_THROW(make({0.5, bad, 0.5, true}, 10), std::invalid_argument);
    EXPECT_THROW(make({0.5, 100.0, bad, true}, 10), std::invalid_argument);
  }
}

}  // namespace
}  // namespace mss::preprocess
