#include "static_gravity_factor.hpp"

#include <cmath>

#include <gtsam/linear/NoiseModel.h>

#include "mss/common/lie.hpp"

namespace mss::backend::detail {

using common::Mat3;
using common::Vec3;
using Mat36 = Eigen::Matrix<double, 3, 6>;

StaticGravityFactor::Projection StaticGravityFactor::projection(const common::Vec3& f_b) {
  const Vec3 u = f_b.normalized();
  // The coordinate axis least aligned with u gives a well-conditioned first tangent.
  Vec3 a = Vec3::UnitX();
  if (std::abs(u.y()) < std::abs(u(0)) && std::abs(u.y()) <= std::abs(u.z())) {
    a = Vec3::UnitY();
  } else if (std::abs(u.z()) < std::abs(u(0))) {
    a = Vec3::UnitZ();
  }
  const Vec3 e1 = u.cross(a).normalized();
  const Vec3 e2 = u.cross(e1);
  Projection P;
  P.row(0) = e1.transpose();
  P.row(1) = e2.transpose();
  return P;
}

namespace {

Eigen::Matrix2d projected_cov(const StaticGravityFactor::Projection& P, const Mat3& cov) {
  const Eigen::Matrix2d c = P * cov * P.transpose();
  const Eigen::Matrix2d sym = 0.5 * (c + c.transpose());
  return sym;
}

}  // namespace

StaticGravityFactor::StaticGravityFactor(gtsam::Key x,
                                         gtsam::Key b,
                                         const common::Vec3& f_b,
                                         const common::Mat3& cov_m2ps4,
                                         const common::Vec3& g_w)
    : Base(
          gtsam::noiseModel::Gaussian::Covariance(projected_cov(projection(f_b), cov_m2ps4)), x, b),
      f_b_(f_b),
      g_w_(g_w),
      P_(projection(f_b)) {}

// Residual and Jacobians: class comment and backend_factors.md section 6c. Right
// perturbation R <- R Exp(dphi): R^T g -> Exp(-dphi) R^T g = R^T g + skew(R^T g) dphi.
gtsam::Vector StaticGravityFactor::evaluateError(const gtsam::Pose3& T_w_b,
                                                 const gtsam::imuBias::ConstantBias& bias,
                                                 boost::optional<gtsam::Matrix&> H_x,
                                                 boost::optional<gtsam::Matrix&> H_b) const {
  const Mat3 R_w_b = T_w_b.rotation().matrix();
  const Vec3 g_b = R_w_b.transpose() * g_w_;  // gravity in b, m/s^2
  const Vec3 e = f_b_ - bias.accelerometer() + g_b;
  if (H_x) {
    Mat36 J = Mat36::Zero();
    J.block<3, 3>(0, 0) = common::skew(g_b);  // [dphi, drho]
    *H_x = P_ * J;
  }
  if (H_b) {
    Mat36 J = Mat36::Zero();
    J.block<3, 3>(0, 0) = -Mat3::Identity();  // [dba, dbg]
    *H_b = P_ * J;
  }
  const Eigen::Vector2d r = P_ * e;
  return r;
}

gtsam::NonlinearFactor::shared_ptr StaticGravityFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new StaticGravityFactor(*this));
}

}  // namespace mss::backend::detail
