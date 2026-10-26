#include "frames.hpp"

#include "mss/common/lie.hpp"

namespace mss::pipeline::detail {

common::PoseCovariance lidar_cov_in_body(const common::PoseCovariance& cov_l,
                                         const common::Pose& T_b_l) {
  return common::transform_covariance(cov_l, T_b_l);
}

common::DegeneracyReport degeneracy_in_body(const common::DegeneracyReport& r_l,
                                            const common::Pose& T_b_l) {
  common::DegeneracyReport r_b = r_l;
  const common::Mat6 adj = common::se3_adj(T_b_l);
  r_b.eigvecs = adj * r_l.eigvecs;
  return r_b;
}

}  // namespace mss::pipeline::detail
