#include <type_traits>

#include <gtest/gtest.h>

#include "mss/common/types.hpp"

namespace mss::common {

// Eigen fixed-size matrices have user-provided copy constructors, so structs that hold them
// are not trivially_copyable; require cheap, non-throwing moves instead. Only pure-scalar
// structs are trivially copyable.
static_assert(std::is_trivially_copyable_v<WheelSample>);
static_assert(std::is_trivially_copyable_v<Timestamp>);
static_assert(std::is_nothrow_move_constructible_v<ImuSample>);
static_assert(std::is_nothrow_move_constructible_v<ImuBias>);
static_assert(std::is_nothrow_move_constructible_v<NavState>);
static_assert(std::is_nothrow_move_constructible_v<Estimate>);
static_assert(std::is_nothrow_move_constructible_v<MatchResult>);
static_assert(std::is_nothrow_move_constructible_v<DegeneracyReport>);
static_assert(std::is_nothrow_move_constructible_v<PreintegratedDelta>);
static_assert(std::is_nothrow_move_constructible_v<GnssMeasurement>);
static_assert(std::is_nothrow_move_constructible_v<OdomDelta>);
static_assert(std::is_nothrow_move_constructible_v<LoopConstraint>);

static_assert(sizeof(ImuSample) == sizeof(Timestamp) + 2 * 3 * sizeof(double));
static_assert(sizeof(WheelSample) == sizeof(Timestamp) + 2 * sizeof(double));
static_assert(sizeof(ImuBias) == 6 * sizeof(double));

TEST(Types, DefaultsAreZeroed) {
  const ImuSample s;
  EXPECT_EQ(s.t.ns, 0);
  EXPECT_EQ(s.acc_b, Vec3::Zero());
  const MatchResult m;
  EXPECT_FALSE(m.converged);
  EXPECT_EQ(m.inliers, 0);
  const NavState x;
  EXPECT_EQ(x.v_w_b, Vec3::Zero());
  EXPECT_EQ(x.T_w_b.R, Mat3::Identity());
  const PreintegratedDelta d;
  EXPECT_EQ(d.dR, Mat3::Identity());
  EXPECT_EQ(d.dt_s, 0.0);
  const DegeneracyReport r;
  for (bool b : r.degenerate) {
    EXPECT_FALSE(b);
  }
}

}  // namespace mss::common
