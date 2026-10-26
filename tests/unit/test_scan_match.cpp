// Unit tests of mss::scan_match (docs/Test_CI_CD.md S-01; docs/derivations/scan_match.md).

#include <algorithm>
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/scan_match/scan_matcher.hpp"

#include "alloc_counter.hpp"
#include "plane_fit.hpp"

namespace mss::scan_match {
namespace {

using common::Mat3;
using common::Pose;
using common::Vec3;
using common::Vec6;

constexpr double kPi = 3.14159265358979323846;

map::NeighborBuffer make_buffer(const std::initializer_list<Vec3>& pts) {
  map::NeighborBuffer buf;
  for (const Vec3& p : pts) {
    buf.p_w[static_cast<std::size_t>(buf.size)] = p;
    buf.dist2[static_cast<std::size_t>(buf.size)] = 0.0F;
    ++buf.size;
  }
  return buf;
}

// Residual r = n_w . (T_w_l p_l - q_w); variable xi = [rho, phi], right perturbation
// T_w_l <- T_w_l * Exp(xi). S-01: analytic J_r_xi vs central differences, rel. error < 1e-6.
TEST(PointToPlane, S01_JacobianMatchesNumerical) {
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u10(-10.0, 10.0);
  std::uniform_real_distribution<double> u5(-5.0, 5.0);
  std::normal_distribution<double> n01(0.0, 1.0);
  constexpr double kH = 1e-6;
  for (int trial = 0; trial < 100; ++trial) {
    const Vec3 p_l(u10(rng), u10(rng), u10(rng));
    const Vec3 q_w(u10(rng), u10(rng), u10(rng));
    const Vec3 n_raw(n01(rng), n01(rng), n01(rng));
    const Vec3 n_w = n_raw.normalized();
    const Vec3 phi(n01(rng), n01(rng), n01(rng));
    const Vec3 t(u5(rng), u5(rng), u5(rng));
    const Pose T_w_l{common::so3_exp(phi), t};
    Vec6 J = Vec6::Zero();
    point_to_plane_residual(p_l, q_w, n_w, T_w_l, &J);
    Vec6 J_num = Vec6::Zero();
    for (int k = 0; k < 6; ++k) {
      const Vec6 e = Vec6::Unit(k) * kH;
      const double rp = point_to_plane_residual(p_l, q_w, n_w, common::boxplus(T_w_l, e), nullptr);
      const double rm = point_to_plane_residual(p_l, q_w, n_w, common::boxplus(T_w_l, -e), nullptr);
      J_num(k) = (rp - rm) / (2.0 * kH);
    }
    const double rel = (J - J_num).norm() / std::max(J_num.norm(), 1e-12);
    EXPECT_LT(rel, 1e-6) << "trial " << trial;
  }
}

TEST(PointToPlane, ResidualIsSignedDistanceAndNullJacobianOk) {
  const Vec3 p_l(1.0, 2.0, 3.0);
  const Pose T_w_l{common::so3_exp(Vec3(0.0, 0.0, kPi / 2.0)), Vec3(0.5, 0.0, 0.0)};
  const Vec3 q_w = Vec3::Zero();
  const Vec3 n_w = Vec3::UnitX();
  Vec6 J = Vec6::Zero();
  const double r = point_to_plane_residual(p_l, q_w, n_w, T_w_l, &J);
  const Vec3 p_w = T_w_l * p_l;
  EXPECT_NEAR(r, p_w.x(), 1e-12);
  EXPECT_EQ(r, point_to_plane_residual(p_l, q_w, n_w, T_w_l, nullptr));
}

TEST(PlaneFit, ExactPlaneRecoversNormalAndZeroRms) {
  const map::NeighborBuffer buf = make_buffer({Vec3(0.0, 0.0, 2.0),
                                               Vec3(0.1, 0.0, 2.0),
                                               Vec3(0.0, 0.1, 2.0),
                                               Vec3(0.1, 0.1, 2.0),
                                               Vec3(0.05, 0.05, 2.0)});
  const std::optional<detail::PlaneFit> fit = detail::fit_plane(buf);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(std::abs(fit->n_w.z()), 1.0, 1e-12);
  const Vec3 centroid(0.05, 0.05, 2.0);
  EXPECT_LT((fit->q_w - centroid).norm(), 1e-12);
  EXPECT_LT(fit->rms_m, 1e-12);
}

TEST(PlaneFit, RmsMatchesOffPlaneSpread) {
  const map::NeighborBuffer buf = make_buffer({Vec3(0.0, 0.0, 0.03),
                                               Vec3(0.2, 0.0, 0.03),
                                               Vec3(0.0, 0.2, 0.03),
                                               Vec3(0.2, 0.2, 0.03),
                                               Vec3(0.0, 0.0, -0.03),
                                               Vec3(0.2, 0.0, -0.03),
                                               Vec3(0.0, 0.2, -0.03),
                                               Vec3(0.2, 0.2, -0.03)});
  const std::optional<detail::PlaneFit> fit = detail::fit_plane(buf);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->rms_m, 0.03, 1e-9);
}

TEST(PlaneFit, FewerThanThreeIsNullopt) {
  EXPECT_FALSE(detail::fit_plane(make_buffer({})).has_value());
  EXPECT_FALSE(detail::fit_plane(make_buffer({Vec3(0.0, 0.0, 0.0)})).has_value());
  EXPECT_FALSE(
      detail::fit_plane(make_buffer({Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)})).has_value());
  EXPECT_TRUE(detail::fit_plane(
                  make_buffer({Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)}))
                  .has_value());
}

}  // namespace
}  // namespace mss::scan_match
