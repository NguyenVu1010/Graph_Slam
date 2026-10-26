#pragma once

#include <Eigen/Core>

#include "mss/common/types.hpp"

// Wheel-odometry residuals with analytic Jacobians, in core types (no GTSAM).
// Derivation: docs/derivations/wheel_odom.md sections 4-5. Tests: O-01, O-02.
//
// Consumer: core/backend wraps these in its GTSAM odometry factors (relative pose from
// WheelIntegrator::take_delta(), and the separate non-holonomic factor that
// backend::Graph::add_odom_factor refers to). backend permutes the [rho, phi] tangent
// used here to GTSAM's [phi, rho] at its boundary. Additive header (plan contract issue 1).
//
// Conventions: T_a_b maps points from b to a; right perturbation T <- T * Exp(xi) with
// xi = [rho, phi] (m, rad); Jacobians are J_<residual>_<variable>.
// All functions: no heap allocation, no logging, no exceptions (hot path safe).
namespace mss::odom {

// Relative-pose residual (O-01), the same as GTSAM BetweenFactor<Pose3>:
//   r = Log(Z^-1 * T_w_bi^-1 * T_w_bj),  Z = T_bi_bj_meas  (6x1 [rho, phi], m / rad).
// Variables: T_w_bi and T_w_bj, both right-perturbed (T <- T * Exp(xi)).
//   J_r_xi_j = Jr^-1(r),  J_r_xi_i = -Jr^-1(r) * Ad(T_w_bj^-1 * T_w_bi).
struct RelativePoseResidual {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::Vec6 r = common::Vec6::Zero();         // [rho, phi], m / rad
  common::Mat6 J_r_xi_i = common::Mat6::Zero();  // right perturbation of T_w_bi
  common::Mat6 J_r_xi_j = common::Mat6::Zero();  // right perturbation of T_w_bj
};

// Errors: none (inputs are poses; a non-finite input gives a non-finite residual, which the
// caller checks). Units: m, rad.
RelativePoseResidual relative_pose_residual(const common::Pose& T_w_bi,
                                            const common::Pose& T_w_bj,
                                            const common::Pose& T_bi_bj_meas);

// Non-holonomic residual (O-02): lateral and vertical velocity of the odometry frame o.
//   v_o = R_b_o^T (R_w_b^T v_w_b + omega_b x p_b_o),  r = [v_o.y, v_o.z]  (m/s).
// Returned UNWHITENED: the backend factor owns the sigma (plan contract issue 2).
// Variables: T_w_b (right perturbation, [rho, phi]) and v_w_b (additive, m/s).
// Constants: omega_b_radps (rad/s, angular rate in b), T_b_o (frames.T_b_o).
//   J_r_xi = S R_b_o^T [0_3, skew(R_w_b^T v_w_b)],  J_r_v = S R_b_o^T R_w_b^T,
//   S = [0 1 0; 0 0 1].
struct NonholonomicResidual {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d r = Eigen::Vector2d::Zero();  // m/s, [lateral, vertical] velocity of o
  Eigen::Matrix<double, 2, 6> J_r_xi = Eigen::Matrix<double, 2, 6>::Zero();  // right pert. T_w_b
  Eigen::Matrix<double, 2, 3> J_r_v = Eigen::Matrix<double, 2, 3>::Zero();   // additive v_w_b
};

// Errors: none (see relative_pose_residual). Units: m/s, rad/s.
NonholonomicResidual nonholonomic_residual(const common::Pose& T_w_b,
                                           const common::Vec3& v_w_b,
                                           const common::Vec3& omega_b_radps,
                                           const common::Pose& T_b_o);

}  // namespace mss::odom
