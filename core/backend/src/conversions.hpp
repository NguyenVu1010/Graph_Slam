#pragma once

// Conversions between mss core types and GTSAM (docs/derivations/backend_factors.md §1).
// mss SE(3) tangent order is [rho, phi]; GTSAM Pose3 is [phi, rho]. P = [0 I; I 0]:
//   xi_g = P xi_m,  Sigma_g = P Sigma_m P^T,  Jacobian columns J_g = J_m P.
// Internal to core/backend (never included by a public header).

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/navigation/ImuBias.h>

#include "mss/common/types.hpp"

namespace mss::backend::detail {

gtsam::Pose3 to_gtsam(const common::Pose& T);
common::Pose from_gtsam(const gtsam::Pose3& T);

// ConstantBias stores [acc, gyro]; ImuBias has named members.
gtsam::imuBias::ConstantBias to_gtsam(const common::ImuBias& b);
common::ImuBias from_gtsam(const gtsam::imuBias::ConstantBias& b);

// P = [0 I; I 0] (its own transpose and inverse).
common::Mat6 permutation_matrix();
// P S P^T: swaps the [rho, phi] and [phi, rho] block orders (an involution).
common::Mat6 permute_cov(const common::Mat6& S);

// Variable keys of keyframe k: pose X(k), velocity V(k), IMU bias B(k).
gtsam::Key pose_key(common::KeyId k);
gtsam::Key vel_key(common::KeyId k);
gtsam::Key bias_key(common::KeyId k);

}  // namespace mss::backend::detail
