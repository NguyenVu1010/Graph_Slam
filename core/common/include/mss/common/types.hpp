#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mss/common/timestamp.hpp"

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

// Identifier of a keyframe / graph node (monotonic, assigned by the pipeline).
using KeyId = std::uint64_t;

// One IMU reading in the body frame b.
struct ImuSample {
  Timestamp t{};               // sensor clock
  Vec3 acc_b = Vec3::Zero();   // m/s^2, specific force in b
  Vec3 gyro_b = Vec3::Zero();  // rad/s, angular rate in b
};

// IMU biases, both expressed in b.
struct ImuBias {
  Vec3 bg = Vec3::Zero();  // rad/s, gyroscope bias
  Vec3 ba = Vec3::Zero();  // m/s^2, accelerometer bias
};

// Navigation state of the body at time t.
struct NavState {
  Timestamp t{};
  Pose T_w_b;                 // body pose in w
  Vec3 v_w_b = Vec3::Zero();  // m/s, body velocity in w
  ImuBias bias;
};

// Output of the estimator for one keyframe.
struct Estimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NavState state;
  PoseCovariance cov = PoseCovariance::Zero();  // 6x6 [rho, phi] of state.T_w_b (m, rad)
  KeyId keyframe_id = 0;
};

// Result of matching one LiDAR scan against the map.
struct MatchResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Pose T_w_l;                                   // LiDAR pose in w
  PoseCovariance cov = PoseCovariance::Zero();  // 6x6 [rho, phi] (m, rad)
  Mat6 H = Mat6::Zero();                        // Gauss-Newton Hessian J^T J, tangent [rho, phi]
  int inliers = 0;                              // correspondences kept
  int iterations = 0;                           // optimiser iterations used
  bool converged = false;
};

// Eigen-analysis of the scan-matching Hessian.
struct DegeneracyReport {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec6 eigvals = Vec6::Zero();       // ascending order
  Mat6 eigvecs = Mat6::Identity();   // column k belongs to eigvals(k); tangent [rho, phi]
  std::array<bool, 6> degenerate{};  // per eigen-direction k
};

// IMU preintegration between two keyframes (increments expressed in the start body b_i).
struct PreintegratedDelta {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Mat3 dR = Mat3::Identity();  // rotation increment R_bi_bj
  Vec3 dv = Vec3::Zero();      // m/s, velocity increment in b_i
  Vec3 dp = Vec3::Zero();      // m, position increment in b_i
  double dt_s = 0.0;           // s, total integrated time
  // 9x9 covariance with block order [phi, p, v] (rad, m, m/s).
  Eigen::Matrix<double, 9, 9> cov = Eigen::Matrix<double, 9, 9>::Zero();
  // Bias Jacobians, J_<quantity>_<bias>, for first-order bias correction.
  Mat3 J_dR_bg = Mat3::Zero();
  Mat3 J_dv_ba = Mat3::Zero();
  Mat3 J_dv_bg = Mat3::Zero();
  Mat3 J_dp_ba = Mat3::Zero();
  Mat3 J_dp_bg = Mat3::Zero();
  ImuBias bias_lin;  // bias at which the delta was linearised
};

// GNSS position fix in the local ENU frame e.
struct GnssMeasurement {
  Timestamp t{};
  Vec3 p_e_g = Vec3::Zero();    // m, antenna position in e
  Mat3 cov = Mat3::Identity();  // m^2, covariance of p_e_g
  std::int8_t status = 0;       // fix status as in NavSatStatus (-1 none, 0 fix, 1 SBAS, 2 GBAS)
};

// Wheel odometry reading.
struct WheelSample {
  Timestamp t{};
  double v_mps = 0.0;        // m/s, forward speed
  double omega_radps = 0.0;  // rad/s, yaw rate
};

// Relative body motion between t0 and t1 from wheel odometry.
struct OdomDelta {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp t0{};
  Timestamp t1{};
  Pose T_b0_b1;                                 // body at t1 expressed in body at t0
  PoseCovariance cov = PoseCovariance::Zero();  // 6x6 [rho, phi] (m, rad)
};

// Loop-closure relative pose between keyframes i and j.
struct LoopConstraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KeyId i = 0;
  KeyId j = 0;
  Pose T_bi_bj;                                 // body j expressed in body i
  PoseCovariance cov = PoseCovariance::Zero();  // 6x6 [rho, phi] (m, rad)
};

// Gravity in w unless IMU initialisation estimates it.
inline const Vec3 kGravityW{0.0, 0.0, -9.80665};

}  // namespace mss::common
