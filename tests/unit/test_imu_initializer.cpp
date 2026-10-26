// mss::imu::ImuInitializer: static gravity alignment and gyro bias (I-01 initialiser part).
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/span.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"
#include "mss/imu/initializer.hpp"

#include "alloc_counter.hpp"
#include "imu_test_utils.hpp"

namespace mss::imu {
namespace {

using common::ImuSample;
using common::Mat3;
using common::Span;
using common::Timestamp;
using common::Vec3;
using test::default_imu_config;

constexpr double kGravityMps2 = 9.80665;

// Static window: acc_b = R_true^T [0, 0, g], gyro_b = bg, t_k = 1 s + k * period.
// 401 samples at 5 ms span exactly 2.0 s (the default init_window_s).
std::vector<ImuSample> static_window(const Mat3& R_true,
                                     const Vec3& bg,
                                     int n = 401,
                                     std::int64_t period_ns = 5'000'000) {
  std::vector<ImuSample> w(static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    ImuSample& s = w[static_cast<std::size_t>(k)];
    s.t = Timestamp{1'000'000'000 + k * period_ns};
    s.acc_b = R_true.transpose() * Vec3(0.0, 0.0, kGravityMps2);
    s.gyro_b = bg;
  }
  return w;
}

Mat3 tilted_rotation() {
  return common::so3_exp(Vec3(0.0, 0.2, 0.0)) * common::so3_exp(Vec3(-0.3, 0.0, 0.0));
}

Vec3 mean_acc(const std::vector<ImuSample>& w) {
  Vec3 m = Vec3::Zero();
  for (const ImuSample& s : w) {
    m += s.acc_b;
  }
  return m / static_cast<double>(w.size());
}

TEST(ImuInitializer, CtorRejectsInvalidConfig) {
  for (const double bad : {0.0, -1.0}) {
    ImuConfig cfg = default_imu_config();
    cfg.init_window_s = bad;
    EXPECT_THROW(ImuInitializer{cfg}, std::invalid_argument);
    cfg = default_imu_config();
    cfg.init_max_acc_std_mps2 = bad;
    EXPECT_THROW(ImuInitializer{cfg}, std::invalid_argument);
    cfg = default_imu_config();
    cfg.init_max_gyro_std_radps = bad;
    EXPECT_THROW(ImuInitializer{cfg}, std::invalid_argument);
    cfg = default_imu_config();
    cfg.gravity_mps2 = bad;
    EXPECT_THROW(ImuInitializer{cfg}, std::invalid_argument);
  }
  EXPECT_NO_THROW(ImuInitializer{default_imu_config()});
}

TEST(ImuInitializer, I01_LevelStaticGivesIdentityAndGyroBias) {
  const Vec3 bg(0.01, -0.02, 0.005);
  const std::vector<ImuSample> w = static_window(Mat3::Identity(), bg);
  ImuInitializer init(default_imu_config());
  const common::Result<InitState> r = init.try_init(Span<const ImuSample>(w));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_LT((r.value().R_w_b - Mat3::Identity()).norm(), 1e-12);
  EXPECT_LT((r.value().bias.bg - bg).norm(), 1e-12);
  EXPECT_TRUE(r.value().bias.ba == Vec3::Zero());
  EXPECT_TRUE(r.value().g_w == Vec3(0.0, 0.0, -kGravityMps2));
}

TEST(ImuInitializer, TiltedStaticRecoversRollPitch) {
  const Mat3 R_true = tilted_rotation();
  const std::vector<ImuSample> w = static_window(R_true, Vec3::Zero());
  ImuInitializer init(default_imu_config());
  const common::Result<InitState> r = init.try_init(Span<const ImuSample>(w));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_LT((r.value().R_w_b - R_true).norm(), 1e-9);
  const Vec3 up = r.value().R_w_b * mean_acc(w);
  EXPECT_LT(up.head<2>().norm(), 1e-9);
  EXPECT_GT(up.z(), 0.0);
}

// Review Focus 5.
TEST(ImuInitializer, UpsideDownAndScaledAccelStillAlignsToPlusZ) {
  ImuInitializer init(default_imu_config());

  std::vector<ImuSample> flipped = static_window(Mat3::Identity(), Vec3::Zero());
  for (ImuSample& s : flipped) {
    s.acc_b = Vec3(0.0, 0.0, -kGravityMps2);
  }
  const common::Result<InitState> rf = init.try_init(Span<const ImuSample>(flipped));
  ASSERT_TRUE(rf.is_ok()) << rf.reason();
  const Vec3 up_f = rf.value().R_w_b * mean_acc(flipped);
  EXPECT_LT(std::abs(up_f.x()), 1e-9);
  EXPECT_LT(std::abs(up_f.y()), 1e-9);
  EXPECT_GT(up_f.z(), 0.0);

  const Mat3 R_true = tilted_rotation();
  const std::vector<ImuSample> plain = static_window(R_true, Vec3::Zero());
  std::vector<ImuSample> scaled = plain;
  for (ImuSample& s : scaled) {
    s.acc_b *= 1.02;
  }
  const common::Result<InitState> rp = init.try_init(Span<const ImuSample>(plain));
  const common::Result<InitState> rs = init.try_init(Span<const ImuSample>(scaled));
  ASSERT_TRUE(rp.is_ok()) << rp.reason();
  ASSERT_TRUE(rs.is_ok()) << rs.reason();
  EXPECT_LT((rs.value().R_w_b - rp.value().R_w_b).norm(), 1e-9);
  EXPECT_EQ(rs.value().g_w.z(), -kGravityMps2);
}

TEST(ImuInitializer, ErrorsInDocumentedOrder) {
  ImuInitializer init(default_imu_config());
  const Vec3 bg(0.01, -0.02, 0.005);
  const std::vector<ImuSample> good = static_window(Mat3::Identity(), bg);

  const std::vector<ImuSample> one(good.begin(), good.begin() + 1);
  EXPECT_EQ(init.try_init(Span<const ImuSample>(one)).reason(), "empty window");

  std::vector<ImuSample> nan_and_dup = good;
  nan_and_dup[10].acc_b.x() = std::numeric_limits<double>::quiet_NaN();
  nan_and_dup[20].t = nan_and_dup[19].t;
  EXPECT_EQ(init.try_init(Span<const ImuSample>(nan_and_dup)).reason(), "non-finite imu sample");

  std::vector<ImuSample> dup = good;
  dup[20].t = dup[19].t;
  EXPECT_EQ(init.try_init(Span<const ImuSample>(dup)).reason(), "imu timestamp not increasing");

  const std::vector<ImuSample> short_w = static_window(Mat3::Identity(), bg, 400);  // 1.995 s
  EXPECT_EQ(init.try_init(Span<const ImuSample>(short_w)).reason(), "window too short");

  std::vector<ImuSample> shaky_acc = good;
  for (std::size_t k = 0; k < shaky_acc.size(); ++k) {
    shaky_acc[k].acc_b.x() += (k % 2 == 0) ? 0.06 : -0.06;
  }
  EXPECT_EQ(init.try_init(Span<const ImuSample>(shaky_acc)).reason(), "not static");

  std::vector<ImuSample> shaky_gyro = good;
  for (std::size_t k = 0; k < shaky_gyro.size(); ++k) {
    shaky_gyro[k].gyro_b.z() = bg.z() + ((k % 2 == 0) ? 0.011 : -0.011);
  }
  EXPECT_EQ(init.try_init(Span<const ImuSample>(shaky_gyro)).reason(), "not static");
}

// "Sample standard deviation" is the n-1 estimator (planner's call).
TEST(ImuInitializer, StdUsesSampleStdPerAxis) {
  std::vector<ImuSample> w = static_window(Mat3::Identity(), Vec3::Zero());
  for (std::size_t k = 0; k < w.size(); ++k) {
    w[k].acc_b.x() = (k % 2 == 0) ? 0.049 : -0.049;  // sample std 0.04906 <= 0.05
  }
  ImuInitializer init(default_imu_config());
  const common::Result<InitState> r = init.try_init(Span<const ImuSample>(w));
  EXPECT_TRUE(r.is_ok()) << r.reason();
}

TEST(ImuInitializer, DoesNotAllocate) {
  const Vec3 bg(0.01, -0.02, 0.005);
  const std::vector<ImuSample> good = static_window(tilted_rotation(), bg);
  const std::vector<ImuSample> one(good.begin(), good.begin() + 1);
  std::vector<ImuSample> nan_w = good;
  nan_w[5].gyro_b.y() = std::numeric_limits<double>::quiet_NaN();
  std::vector<ImuSample> dup = good;
  dup[7].t = dup[6].t;
  const std::vector<ImuSample> short_w = static_window(Mat3::Identity(), bg, 400);
  std::vector<ImuSample> shaky = good;
  for (std::size_t k = 0; k < shaky.size(); ++k) {
    shaky[k].acc_b.y() += (k % 2 == 0) ? 0.06 : -0.06;
  }
  const std::array<const std::vector<ImuSample>*, 5> bad = {&one, &nan_w, &dup, &short_w, &shaky};
  ImuInitializer init(default_imu_config());
  int n_ok = 0;
  int n_err = 0;

  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    n_ok += init.try_init(Span<const ImuSample>(good)).is_ok() ? 1 : 0;
    for (const std::vector<ImuSample>* w : bad) {
      n_err += init.try_init(Span<const ImuSample>(*w)).is_ok() ? 0 : 1;
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0u);
  EXPECT_EQ(n_ok, 1);
  EXPECT_EQ(n_err, 5);
}

}  // namespace
}  // namespace mss::imu
