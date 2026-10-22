#include <cstddef>
#include <string>

#include <gtest/gtest.h>

#include "mss/common/result.hpp"

#include "alloc_counter.hpp"

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

TEST(Result, ErrorFromBuiltString) {
  const std::string key = "imu.rate_hz";
  const Result<int> r = Result<int>::error(key + ": must be > 0");
  EXPECT_FALSE(r.is_ok());
  EXPECT_EQ(r.reason(), "imu.rate_hz: must be > 0");
}

TEST(Status, OkAndError) {
  EXPECT_TRUE(Status::ok());
  EXPECT_FALSE(Status::error("imu gap"));
  EXPECT_TRUE(Status::ok().reason().empty());
}

TEST(Status, LiteralAndOwnedOverloads) {
  const Status lit = Status::error("imu gap");  // const char* overload
  EXPECT_FALSE(lit.is_ok());
  EXPECT_EQ(lit.reason(), "imu gap");
  const Status owned = Status::error(std::string("bad ") + "gnss");
  EXPECT_FALSE(owned.is_ok());
  EXPECT_EQ(owned.reason(), "bad gnss");
  const Status copy = lit;  // copies keep the literal pointer
  EXPECT_EQ(copy.reason(), "imu gap");
  EXPECT_FALSE(Status::error("").is_ok());  // an empty literal is still an error
  EXPECT_EQ(Status::error(static_cast<const char*>(nullptr)).reason(), "unknown error");
}

TEST(Status, LiteralErrorDoesNotAllocate) {
  AllocCounter::reset();
  const Status s = Status::error("imu timestamp not increasing");
  const bool is_ok = s.is_ok();
  const Status copy = s;
  const Result<int> r = Result<int>::error("imu queue full, dropped oldest");
  const bool r_ok = r.is_ok();
  const Status ok = Status::ok();
  const bool ok_ok = ok.is_ok();
  const std::size_t allocs = AllocCounter::count();
  EXPECT_EQ(allocs, 0U);
  EXPECT_FALSE(is_ok);
  EXPECT_FALSE(copy.is_ok());
  EXPECT_FALSE(r_ok);
  EXPECT_TRUE(ok_ok);
}

}  // namespace mss::common
