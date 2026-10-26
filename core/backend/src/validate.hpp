#pragma once

#include "mss/backend/config.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"

namespace mss::backend::detail {

// Checks the Graph constructor inputs (graph.hpp lists the conditions). Shared by the
// GTSAM graph and the stub so both reject the same configs.
// Errors: throws std::invalid_argument naming the field, e.g.
//   "backend.prior_pos_sigma_m must be > 0".
void validate(const BackendConfig& cfg, const imu::ImuConfig& imu_cfg, const common::Vec3& p_b_g);

}  // namespace mss::backend::detail
