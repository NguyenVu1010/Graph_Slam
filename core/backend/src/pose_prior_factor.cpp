#include "pose_prior_factor.hpp"

#include "mss/odom/odom_residual.hpp"

#include "conversions.hpp"

namespace mss::backend::detail {

using common::Mat6;
using common::Vec6;

PosePriorFactor::PosePriorFactor(gtsam::Key x,
                                 const common::Pose& prior,
                                 const gtsam::SharedNoiseModel& noise)
    : Base(noise, x), prior_(prior) {}

// Residual e = P r_m, r_m = Log(X0^-1 X) ([rho, phi]): the relative-pose residual
// (odom O-01) with T_w_bi = X0, T_w_bj = X and Z = Identity. Variable X, right perturbation;
// GTSAM tangent xi_g = [dphi, drho] = P xi_m, so H = P J_r_xi_j P (as Se3RelativePoseFactor).
gtsam::Vector PosePriorFactor::evaluateError(const gtsam::Pose3& X,
                                             boost::optional<gtsam::Matrix&> H_x) const {
  const odom::RelativePoseResidual res =
      odom::relative_pose_residual(prior_, from_gtsam(X), common::Pose{});
  if (H_x) {
    const Mat6 H = permute_cov(res.J_r_xi_j);  // P J P^T with P = P^T (conversions.hpp)
    *H_x = H;
  }
  Vec6 e;
  e << res.r.tail<3>(), res.r.head<3>();  // [phi, rho]
  return e;
}

gtsam::NonlinearFactor::shared_ptr PosePriorFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new PosePriorFactor(*this));
}

}  // namespace mss::backend::detail
