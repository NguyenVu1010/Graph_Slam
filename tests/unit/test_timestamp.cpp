#include <gtest/gtest.h>

#include "mss/common/timestamp.hpp"

namespace mss::common {

TEST(Timestamp, ComparesAsInteger) {
  const Timestamp a{10};
  const Timestamp b{11};
  EXPECT_LT(a, b);
  EXPECT_NE(a, b);
  EXPECT_EQ(a, Timestamp{10});
}

TEST(Timestamp, SecondsBetweenIsSigned) {
  const Timestamp t0 = timestamp_from_sec_nsec(1, 500000000);
  const Timestamp t1 = timestamp_from_sec_nsec(2, 0);
  EXPECT_DOUBLE_EQ(seconds_between(t0, t1), 0.5);
  EXPECT_DOUBLE_EQ(seconds_between(t1, t0), -0.5);
}

}  // namespace mss::common
