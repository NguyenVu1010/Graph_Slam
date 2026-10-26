#include "conversions.hpp"

#include <gtsam/inference/Symbol.h>

namespace mss::backend::detail {

gtsam::Pose3 to_gtsam(const common::Pose& T) {
  return gtsam::Pose3(gtsam::Rot3(T.R), gtsam::Point3(T.p));
}

common::Pose from_gtsam(const gtsam::Pose3& T) {
  common::Pose out;
  out.R = T.rotation().matrix();
  out.p = T.translation();
  return out;
}

gtsam::imuBias::ConstantBias to_gtsam(const common::ImuBias& b) {
  return gtsam::imuBias::ConstantBias(b.ba, b.bg);  // order: accelerometer, gyroscope
}

common::ImuBias from_gtsam(const gtsam::imuBias::ConstantBias& b) {
  common::ImuBias out;
  out.ba = b.accelerometer();
  out.bg = b.gyroscope();
  return out;
}

common::Mat6 permutation_matrix() {
  common::Mat6 P = common::Mat6::Zero();
  P.topRightCorner<3, 3>().setIdentity();
  P.bottomLeftCorner<3, 3>().setIdentity();
  return P;
}

common::Mat6 permute_cov(const common::Mat6& S) {
  common::Mat6 out;
  out.topLeftCorner<3, 3>() = S.bottomRightCorner<3, 3>();
  out.topRightCorner<3, 3>() = S.bottomLeftCorner<3, 3>();
  out.bottomLeftCorner<3, 3>() = S.topRightCorner<3, 3>();
  out.bottomRightCorner<3, 3>() = S.topLeftCorner<3, 3>();
  return out;
}

gtsam::Key pose_key(common::KeyId k) {
  return gtsam::symbol_shorthand::X(k);
}
gtsam::Key vel_key(common::KeyId k) {
  return gtsam::symbol_shorthand::V(k);
}
gtsam::Key bias_key(common::KeyId k) {
  return gtsam::symbol_shorthand::B(k);
}

}  // namespace mss::backend::detail
