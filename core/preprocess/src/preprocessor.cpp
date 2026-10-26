#include "mss/preprocess/preprocessor.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "mss/common/span.hpp"
#include "mss/common/types.hpp"

namespace mss::preprocess {

namespace {
void require_finite(double v, const char* key) {
  if (!std::isfinite(v)) {
    throw std::invalid_argument(std::string(key) + " must be finite");
  }
}

// Throws std::invalid_argument naming the YAML key; runs before any member allocates.
const PreprocessConfig& validated(const PreprocessConfig& cfg, std::size_t max_points) {
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
  return cfg;
}
}  // namespace

// Eigen members are passed by const& (CLAUDE.md); the signature is the frozen contract.
Preprocessor::Preprocessor(const PreprocessConfig& cfg,
                           const common::Pose& T_b_l,  // NOLINT(modernize-pass-by-value)
                           std::size_t max_points)
    : cfg_(validated(cfg, max_points)), T_b_l_(T_b_l), voxels_(max_points) {}

common::Status Preprocessor::process(const common::ScanFrame& in,
                                     const imu::ImuTrajectory& traj,
                                     common::ScanFrame& out) {
  out.clear();
  if (out.capacity() < in.size()) {
    return common::Status::error("output capacity too small");
  }
  if (cfg_.deskew_enabled && traj.size == 0) {
    return common::Status::error("empty imu trajectory");
  }
  out.t_start = in.t_start;
  const bool downsample = cfg_.downsample_voxel_m > 0.0;
  if (cfg_.deskew_enabled) {
    deskew_.build(traj, T_b_l_);
  }
  if (downsample) {
    voxels_.clear();
  }

  const common::Span<const float> xyz_in = in.xyz();
  const common::Span<const float> intensity_in = in.intensity();
  const common::Span<const std::uint32_t> t_offset_in = in.t_offset_ns();
  const common::Span<const std::uint16_t> ring_in = in.ring();
  float* const xyz_out = out.xyz_mut();
  float* const intensity_out = out.intensity_mut();
  std::uint32_t* const t_offset_out = out.t_offset_ns_mut();
  std::uint16_t* const ring_out = out.ring_mut();

  std::size_t n = 0;
  for (std::size_t i = 0; i < in.size(); ++i) {
    float x = xyz_in[3 * i];
    float y = xyz_in[3 * i + 1];
    float z = xyz_in[3 * i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const double xd = x;  // widen to double
    const double yd = y;  // widen to double
    const double zd = z;  // widen to double
    const double range_m = std::sqrt(xd * xd + yd * yd + zd * zd);
    if (range_m < cfg_.min_range_m || range_m > cfg_.max_range_m) {
      continue;
    }
    if (cfg_.deskew_enabled) {
      const common::Timestamp t{in.t_start.ns + static_cast<std::int64_t>(t_offset_in[i])};
      const common::Vec3 p_l0 = deskew_.T_l0_l(t) * common::Vec3(xd, yd, zd);
      x = static_cast<float>(p_l0.x());
      y = static_cast<float>(p_l0.y());
      z = static_cast<float>(p_l0.z());
    }
    // The key comes from the float coordinates written to out (derivation section 5).
    if (downsample && !voxels_.insert(detail::voxel_key(x, y, z, cfg_.downsample_voxel_m))) {
      continue;
    }
    xyz_out[3 * n] = x;
    xyz_out[3 * n + 1] = y;
    xyz_out[3 * n + 2] = z;
    intensity_out[n] = intensity_in[i];
    t_offset_out[n] = t_offset_in[i];
    ring_out[n] = ring_in[i];
    ++n;
  }
  out.set_size(n);
  return common::Status::ok();
}

}  // namespace mss::preprocess
