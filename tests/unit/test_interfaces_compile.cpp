// Compile-only check of the module interface headers (contracts for parallel
// development). Includes every header and asserts a few type properties. It calls no
// declared-but-undefined function: the modules have no sources yet.
#include <type_traits>

#include <gtest/gtest.h>

#include "mss/backend/config.hpp"
#include "mss/backend/graph.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/degeneracy/config.hpp"
#include "mss/degeneracy/degeneracy.hpp"
#include "mss/gnss/config.hpp"
#include "mss/gnss/gnss.hpp"
#include "mss/imu/config.hpp"
#include "mss/imu/initializer.hpp"
#include "mss/imu/preintegrator.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/loop/config.hpp"
#include "mss/loop/loop_detector.hpp"
#include "mss/loop/loop_selector.hpp"
#include "mss/map/config.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/odom/config.hpp"
#include "mss/odom/wheel_integrator.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/estimator.hpp"
#include "mss/preprocess/config.hpp"
#include "mss/preprocess/preprocessor.hpp"
#include "mss/scan_match/config.hpp"
#include "mss/scan_match/scan_matcher.hpp"

static_assert(!std::is_copy_constructible_v<mss::common::ScanFrame>);
static_assert(sizeof(mss::imu::ImuTrajectory) > 0);
static_assert(mss::map::NeighborBuffer::kMaxNeighbors == 32);
static_assert(mss::imu::ImuTrajectory::kMaxImuPerScan == 128);
static_assert(mss::gnss::GnssAligner::kMinPairsForAlignment == 10);
static_assert(!std::is_copy_constructible_v<mss::backend::Graph>);
static_assert(std::is_nothrow_move_constructible_v<mss::backend::Graph>);

TEST(InterfacesCompile, PlainStructsConstruct) {
  mss::imu::ImuTrajectory traj{};
  EXPECT_EQ(traj.size, 0U);
  EXPECT_EQ(traj.samples.size(), mss::imu::ImuTrajectory::kMaxImuPerScan);

  mss::map::NeighborBuffer nb{};
  EXPECT_EQ(nb.size, 0);
  EXPECT_EQ(nb.p_w.size(), static_cast<std::size_t>(mss::map::NeighborBuffer::kMaxNeighbors));

  const mss::gnss::Lla origin{52.0, 13.0, 34.5};
  EXPECT_DOUBLE_EQ(origin.lat_deg, 52.0);
  EXPECT_DOUBLE_EQ(origin.lon_deg, 13.0);
  EXPECT_DOUBLE_EQ(origin.alt_m, 34.5);
}
