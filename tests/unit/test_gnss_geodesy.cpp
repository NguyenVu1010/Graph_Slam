#include <algorithm>
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "mss/common/types.hpp"
#include "mss/gnss/gnss.hpp"

#include "alloc_counter.hpp"

namespace {

using mss::common::Vec3;
using mss::gnss::enu_to_lla;
using mss::gnss::Lla;
using mss::gnss::lla_to_enu;

// Same origin as tests/python/test_geodesy.py.
const Lla kOrigin{21.0278, 105.8342, 12.0};

void expect_lla_near(const Lla& got, const Lla& want) {
  EXPECT_LT(std::abs(got.lat_deg - want.lat_deg), 1e-9);
  EXPECT_LT(std::abs(got.lon_deg - want.lon_deg), 1e-9);
  EXPECT_LT(std::abs(got.alt_m - want.alt_m), 1e-4);
}

TEST(GnssGeodesy, G01_RoundTripUnder1mm) {  // Test_CI_CD G-01; same box as test_geodesy.py
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> u(-5000.0, 5000.0);
  double max_err_m = 0.0;
  for (int i = 0; i < 1000; ++i) {
    const Vec3 p_e(u(rng), u(rng), u(rng));
    const Vec3 back = lla_to_enu(enu_to_lla(p_e, kOrigin), kOrigin);
    max_err_m = std::max(max_err_m, (back - p_e).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_err_m, 1e-3);
}

TEST(GnssGeodesy, MatchesPythonReference) {
  // Values from sim/mss_sim/geodesy.py at ff28e96; C++ must agree with Python (G-01).
  // Tolerances (planner's call) are far below 1 mm.
  {
    const Vec3 got = lla_to_enu(Lla{21.0368, 105.8442, 40.0}, kOrigin);
    const Vec3 want(1039.4554685027003, 996.4958544532689, 27.83706901384857);
    EXPECT_LT((got - want).norm(), 1e-4);
  }
  {
    const Vec3 got = lla_to_enu(Lla{20.9, 105.7, -5.0}, kOrigin);
    const Vec3 want(-13962.076325535796, -14143.68619348998, -48.042805916612);
    EXPECT_LT((got - want).norm(), 1e-4);
  }
  expect_lla_near(enu_to_lla(Vec3(1234.5, -2345.25, 67.75), kOrigin),
                  Lla{21.00661751722635, 105.84607394566848, 80.3029325408861});
  expect_lla_near(enu_to_lla(Vec3(-4000.0, 3000.0, -20.0), Lla{-33.9, 151.2, 100.0}),
                  Lla{-33.87294632184269, 151.156767638509, 81.96102854702622});
  expect_lla_near(enu_to_lla(Vec3(500.0, 500.0, 10.0), Lla{89.9, 0.0, 0.0}),
                  Lla{89.90437167658892, 2.6830860152486298, 10.039064920507371});
  expect_lla_near(enu_to_lla(Vec3(500.0, 0.0, 0.0), Lla{10.0, 179.999, 0.0}),
                  Lla{9.999999968760397, -179.9964395941349, 0.019596220925450325});
}

TEST(GnssGeodesy, OriginMapsToZero) {
  EXPECT_LT(lla_to_enu(kOrigin, kOrigin).norm(), 1e-6);
}

TEST(GnssGeodesy, EastNorthUpAxes) {
  const Lla east = enu_to_lla(Vec3(100.0, 0.0, 0.0), kOrigin);
  EXPECT_GT(east.lon_deg, kOrigin.lon_deg);
  EXPECT_LT(std::abs(east.lat_deg - kOrigin.lat_deg), 1e-8);
  EXPECT_LT(std::abs(east.alt_m - kOrigin.alt_m), 1e-3);

  const Lla north_up = enu_to_lla(Vec3(0.0, 100.0, 50.0), kOrigin);
  EXPECT_GT(north_up.lat_deg, kOrigin.lat_deg);
  EXPECT_LT(std::abs(north_up.lon_deg - kOrigin.lon_deg), 1e-8);
  EXPECT_LT(std::abs(north_up.alt_m - (kOrigin.alt_m + 50.0)), 1e-3);
}

TEST(GnssGeodesy, RoundTripNearPoleAndAntimeridian) {  // Review Focus 5
  const Lla origins[] = {{89.9, 0.0, 0.0}, {-89.9, 45.0, 100.0}, {10.0, 179.999, 0.0}};
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-2000.0, 2000.0);
  for (const Lla& origin : origins) {
    double max_err_m = 0.0;
    for (int i = 0; i < 100; ++i) {
      const Vec3 p_e(u(rng), u(rng), u(rng));
      const Lla lla = enu_to_lla(p_e, origin);
      EXPECT_GT(lla.lon_deg, -180.0);
      EXPECT_LE(lla.lon_deg, 180.0);
      const Vec3 back = lla_to_enu(lla, origin);
      max_err_m = std::max(max_err_m, (back - p_e).cwiseAbs().maxCoeff());
    }
    EXPECT_LT(max_err_m, 1e-3) << "origin lat " << origin.lat_deg << " lon " << origin.lon_deg;
  }
}

TEST(GnssGeodesy, NoAllocation) {
  double sink = 0.0;
  AllocCounter::reset();
  {
    EigenMallocGuard guard;
    for (int i = 0; i < 100; ++i) {
      const Vec3 p_e(static_cast<double>(i), -2.0 * i, 0.5 * i);
      const Lla lla = enu_to_lla(p_e, kOrigin);
      const Vec3 back = lla_to_enu(lla, kOrigin);
      sink += back.x();
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0u);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
