#include "mss/backend/graph.hpp"

#include <gtsam/config.h>

// TEMPORARY skeleton of the GTSAM build of mss::backend::Graph (filled in by plan Task 4).
#ifndef GTSAM_POSE3_EXPMAP
#error "backend assumes Pose3::retract = T * Expmap (docs/derivations/backend_factors.md section 1)"
#endif

#include <limits>
#include <memory>

#include "conversions.hpp"
#include "validate.hpp"

namespace mss::backend {

struct Graph::Impl {
  common::KeyId next_id_ = 0;
};

Graph::Graph(const BackendConfig& cfg, const imu::ImuConfig& imu_cfg, const common::Vec3& p_b_g) {
  detail::validate(cfg, imu_cfg, p_b_g);
  impl_ = std::make_unique<Impl>();
}

Graph::~Graph() = default;
Graph::Graph(Graph&&) noexcept = default;
Graph& Graph::operator=(Graph&&) noexcept = default;

common::KeyId Graph::add_keyframe(common::Timestamp /*t*/, const common::NavState& /*x0*/) {
  return impl_->next_id_++;
}

void Graph::add_imu_factor(common::KeyId /*i*/,
                           common::KeyId /*j*/,
                           const common::PreintegratedDelta& /*delta*/) {}

void Graph::add_lidar_factor(common::KeyId /*i*/,
                             common::KeyId /*j*/,
                             const common::Pose& /*T_bi_bj*/,
                             const common::PoseCovariance& /*cov*/,
                             const common::DegeneracyReport* /*degeneracy*/) {}

void Graph::add_gnss_factor(common::KeyId /*k*/,
                            const common::GnssMeasurement& /*m*/,
                            const common::Pose& /*T_e_w*/) {}

void Graph::add_odom_factor(common::KeyId /*i*/,
                            common::KeyId /*j*/,
                            const common::OdomDelta& /*odom*/) {}

void Graph::add_loop_factor(const common::LoopConstraint& /*lc*/) {}

common::Result<common::Estimate> Graph::optimize() {
  return common::Result<common::Estimate>::error("empty graph");
}

common::PoseCovariance Graph::marginal_cov(common::KeyId /*k*/) const {
  return common::PoseCovariance::Constant(std::numeric_limits<double>::quiet_NaN());
}

common::Result<common::NavState> Graph::keyframe_state(common::KeyId /*k*/) const {
  return common::Result<common::NavState>::error("empty graph");
}

}  // namespace mss::backend
