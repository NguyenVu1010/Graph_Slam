#include "mss/common/lie.hpp"

#include <Eigen/Geometry>
#include <sophus/so3.hpp>

namespace mss::common {
namespace {

// Same as the SO3(Matrix3) constructor, minus the orthogonality assertion: Sophus stores
// a unit quaternion, and the quaternion constructor normalises it.
Sophus::SO3d so3_from_matrix(const Mat3& R) {
  return Sophus::SO3d(Eigen::Quaterniond(R));
}

}  // namespace

Mat3 skew(const Vec3& v) {
  return Sophus::SO3d::hat(v);
}

Mat3 so3_exp(const Vec3& phi) {
  return Sophus::SO3d::exp(phi).matrix();
}

Vec3 so3_log(const Mat3& R) {
  return so3_from_matrix(R).log();
}

Mat3 so3_jr(const Vec3& phi) {
  return Sophus::SO3d::leftJacobian(Vec3(-phi));
}

Mat3 so3_jr_inv(const Vec3& phi) {
  return Sophus::SO3d::leftJacobianInverse(Vec3(-phi));
}

Mat3 so3_jl(const Vec3& phi) {
  return Sophus::SO3d::leftJacobian(phi);
}

Mat3 so3_jl_inv(const Vec3& phi) {
  return Sophus::SO3d::leftJacobianInverse(phi);
}

// Sophus SE3 tangent is [translation, rotation] = [rho, phi], the mss order.
Pose se3_exp(const Vec6& xi) {
  return from_sophus(Sophus::SE3d::exp(xi));
}

Vec6 se3_log(const Pose& T) {
  return to_sophus(T).log();
}

// Sophus SE3::Adj() = [R, hat(t) R; 0, R], acting on [rho, phi].
Mat6 se3_adj(const Pose& T) {
  return to_sophus(T).Adj();
}

Pose boxplus(const Pose& T, const Vec6& delta) {
  return T * se3_exp(delta);
}

Vec6 boxminus(const Pose& a, const Pose& b) {
  return se3_log(b.inverse() * a);
}

PoseCovariance transform_covariance(const PoseCovariance& cov_x, const Pose& T_y_x) {
  const Mat6 Ad = se3_adj(T_y_x);
  const Mat6 cov_y = Ad * cov_x * Ad.transpose();
  return PoseCovariance(0.5 * (cov_y + cov_y.transpose()));
}

Sophus::SE3d to_sophus(const Pose& T) {
  return Sophus::SE3d(so3_from_matrix(T.R), T.p);
}

Pose from_sophus(const Sophus::SE3d& T) {
  return Pose{T.rotationMatrix(), T.translation()};
}

}  // namespace mss::common
