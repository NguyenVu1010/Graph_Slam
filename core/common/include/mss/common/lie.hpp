#pragma once

#include <sophus/se3.hpp>

#include "mss/common/types.hpp"

// Lie group operations on SO(3) and SE(3). The only place in mss that includes Sophus.
// Conventions (docs/derivations/lie_conventions.md, docs/conventions.md):
//   - right perturbation: T <- T * Exp(delta), R <- R * Exp(dphi);
//   - SE(3) tangent order [rho, phi] (translation, rotation) for every vector,
//     Jacobian and covariance. GTSAM Pose3 uses [phi, rho]; core/backend permutes.
namespace mss::common {

// skew(v) * u == v.cross(u).
Mat3 skew(const Vec3& v);

// SO(3) exponential and logarithm. so3_log returns phi with |phi| in [0, pi].
Mat3 so3_exp(const Vec3& phi);
Vec3 so3_log(const Mat3& R);

// Right Jacobian: Exp(phi + dphi) ~= Exp(phi) * Exp(so3_jr(phi) * dphi).
Mat3 so3_jr(const Vec3& phi);
Mat3 so3_jr_inv(const Vec3& phi);
// Left Jacobian: Exp(phi + dphi) ~= Exp(so3_jl(phi) * dphi) * Exp(phi). Jl(phi) = Jr(-phi).
Mat3 so3_jl(const Vec3& phi);
Mat3 so3_jl_inv(const Vec3& phi);

// SE(3) exponential and logarithm, xi = [rho, phi].
Pose se3_exp(const Vec6& xi);
Vec6 se3_log(const Pose& T);

// Adjoint acting on [rho, phi]: T * Exp(xi) * T^-1 = Exp(se3_adj(T) * xi).
// Ad = [R, skew(p) R; 0, R].
Mat6 se3_adj(const Pose& T);

// Right perturbation: boxplus(T, delta) = T * Exp(delta).
Pose boxplus(const Pose& T, const Vec6& delta);
// boxminus(a, b) = Log(b^-1 * a), so boxminus(boxplus(T, d), T) == d.
Vec6 boxminus(const Pose& a, const Pose& b);

// Covariance of a [rho, phi] perturbation expressed in frame x, re-expressed in frame y:
// Ad(T_y_x) * cov_x * Ad(T_y_x)^T. The result is exactly symmetric.
PoseCovariance transform_covariance(const PoseCovariance& cov_x, const Pose& T_y_x);

// Conversions. to_sophus re-normalises the rotation through a unit quaternion instead of
// aborting (Sophus' SO3(Matrix3) constructor asserts orthogonality).
Sophus::SE3d to_sophus(const Pose& T);
Pose from_sophus(const Sophus::SE3d& T);

}  // namespace mss::common
