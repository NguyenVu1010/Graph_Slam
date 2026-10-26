#include "gnss_factor.hpp"

#include <gtsam/linear/NoiseModel.h>

#include "mss/common/lie.hpp"

namespace mss::backend::detail {

using common::Mat3;
using common::Vec3;

GnssPositionFactor::GnssPositionFactor(gtsam::Key x,
                                       const common::Vec3& p_e_g,
                                       const common::Mat3& cov_m2,
                                       const common::Pose& T_e_w,
                                       const common::Vec3& p_b_g)
    : Base(gtsam::noiseModel::Gaussian::Covariance(cov_m2), x),
      p_e_g_(p_e_g),
      T_e_w_(T_e_w),
      p_b_g_(p_b_g) {}

// Residual r = R_e_w (R_w_b p_b_g + p_w_b) + p_e_w - p_e_g (m); variable T_w_b with the
// GTSAM right perturbation [dphi, drho]:
//   d r / d[dphi, drho] = [-R_e_w R_w_b skew(p_b_g), R_e_w R_w_b]  (section 6).
gtsam::Vector GnssPositionFactor::evaluateError(const gtsam::Pose3& T_w_b,
                                                boost::optional<gtsam::Matrix&> H_x) const {
  const Mat3 R_w_b = T_w_b.rotation().matrix();
  const Vec3 p_w_b = T_w_b.translation();
  const Vec3 r = T_e_w_.R * (R_w_b * p_b_g_ + p_w_b) + T_e_w_.p - p_e_g_;
  if (H_x) {
    const Mat3 R_e_b = T_e_w_.R * R_w_b;
    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>() = -R_e_b * common::skew(p_b_g_);
    J.rightCols<3>() = R_e_b;
    *H_x = J;
  }
  return r;
}

gtsam::NonlinearFactor::shared_ptr GnssPositionFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new GnssPositionFactor(*this));
}

}  // namespace mss::backend::detail
