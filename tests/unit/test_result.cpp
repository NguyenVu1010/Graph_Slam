#include <gtest/gtest.h>

#include "mss/common/result.hpp"

namespace mss::common {

TEST(Result, OkCarriesValue) {
  const Result<int> r = Result<int>::ok(42);
  ASSERT_TRUE(r);
  EXPECT_EQ(r.value(), 42);
  EXPECT_TRUE(r.reason().empty());
}

TEST(Result, ErrorCarriesReason) {
  const Result<int> r = Result<int>::error("too few points");
  EXPECT_FALSE(r);
  EXPECT_EQ(r.reason(), "too few points");
}

TEST(Status, OkAndError) {
  EXPECT_TRUE(Status::ok());
  EXPECT_FALSE(Status::error("imu gap"));
}

}  // namespace mss::common
