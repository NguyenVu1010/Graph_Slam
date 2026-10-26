#include "mss/preprocess/preprocessor.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace mss::preprocess {

namespace {
void require_finite(double v, const char* key) {
  if (!std::isfinite(v)) {
    throw std::invalid_argument(std::string(key) + " must be finite");
  }
}
}  // namespace

Preprocessor::Preprocessor(const PreprocessConfig& cfg,
                           const common::Pose& T_b_l,
                           std::size_t max_points)
    : cfg_(cfg), T_b_l_(T_b_l) {
  require_finite(cfg.min_range_m, "preprocess.min_range_m");
  require_finite(cfg.max_range_m, "preprocess.max_range_m");
  require_finite(cfg.downsample_voxel_m, "preprocess.downsample_voxel_m");
  if (cfg.min_range_m < 0.0) {
    throw std::invalid_argument("preprocess.min_range_m must be >= 0");
  }
  if (cfg.max_range_m <= cfg.min_range_m) {
    throw std::invalid_argument("preprocess.max_range_m must be > preprocess.min_range_m");
  }
  if (cfg.downsample_voxel_m < 0.0) {
    throw std::invalid_argument("preprocess.downsample_voxel_m must be >= 0");
  }
  if (max_points == 0) {
    throw std::invalid_argument("realtime.max_points_per_scan (max_points) must be > 0");
  }
}

common::Status Preprocessor::process(const common::ScanFrame& /*in*/,
                                     const imu::ImuTrajectory& /*traj*/,
                                     common::ScanFrame& /*out*/) {
  return common::Status::error("not implemented");
}

}  // namespace mss::preprocess
