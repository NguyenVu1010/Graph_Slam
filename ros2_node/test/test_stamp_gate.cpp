// Unit tests of mss_ros2::StampGate (backlog R4: only a used message advances the stamp).
#include <gtest/gtest.h>

#include "mss_ros2/stamp_gate.hpp"

namespace {

using mss::common::Timestamp;
using mss_ros2::StampGate;

TEST(StampGate, FirstStampIsNewAndCheckingDoesNotAdvance) {
  StampGate g;
  EXPECT_FALSE(g.has_last());
  EXPECT_TRUE(g.is_new(Timestamp{100}));
  EXPECT_TRUE(g.is_new(Timestamp{100}));  // still new: is_new() never commits
  EXPECT_FALSE(g.has_last());
}

// The on_lidar sequence: a malformed cloud with a late stamp is checked but not committed
// (its conversion failed), so the real publisher's earlier-stamped scan is still accepted.
TEST(StampGate, UncommittedLateStampDoesNotBlockEarlierGoodScan) {
  StampGate g;
  g.commit(Timestamp{1000});
  ASSERT_TRUE(g.is_new(Timestamp{5000}));  // malformed cloud passes the check, conversion fails
  // no commit
  EXPECT_TRUE(g.is_new(Timestamp{2000}));  // good scan, earlier than the malformed one
  g.commit(Timestamp{2000});
  EXPECT_FALSE(g.is_new(Timestamp{2000}));  // equal is not new
  EXPECT_FALSE(g.is_new(Timestamp{1500}));
  EXPECT_TRUE(g.is_new(Timestamp{2001}));
  EXPECT_EQ(g.last().ns, 2000);
}

}  // namespace
