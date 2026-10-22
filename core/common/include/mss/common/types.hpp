#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// Frame conventions: see docs/conventions.md.
//   T_w_b maps points from b to w:  p_w = T_w_b * p_b.
//   Right perturbation on SE(3):    T <- T * Exp(delta), delta = [rho, phi].
//   Frames: w world/map, b body = IMU, l LiDAR, e ENU, g GNSS antenna.
namespace mss::common {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

// Rigid transform. Stored as rotation + translation to keep the SE(3) tangent
// order [rho, phi] explicit at the API boundary.
struct Pose {
  Mat3 R = Mat3::Identity();  // R_a_b
  Vec3 p = Vec3::Zero();      // p_a_b: origin of b expressed in a

  Vec3 operator*(const Vec3& p_b) const { return R * p_b + p; }
  Pose operator*(const Pose& T_b_c) const { return Pose{R * T_b_c.R, R * T_b_c.p + p}; }
  Pose inverse() const { return Pose{R.transpose(), -(R.transpose() * p)}; }
};

// 6x6 covariance in the tangent order [rho, phi].
using PoseCovariance = Mat6;

struct ImuSample {
  Vec3 acc_b = Vec3::Zero();   // m/s^2, specific force in b
  Vec3 gyro_b = Vec3::Zero();  // rad/s, angular rate in b
};

// Gravity in w unless IMU initialisation estimates it.
inline const Vec3 kGravityW{0.0, 0.0, -9.80665};

}  // namespace mss::common
