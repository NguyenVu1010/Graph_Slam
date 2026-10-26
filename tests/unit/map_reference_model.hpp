#pragma once

// Brute-force reference model of mss::map::VoxelMap for tests. It applies the same key,
// float rounding, ring, drop and prune rules (docs/derivations/map.md) but stores voxels in
// a std::map and searches by scanning every stored point.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <vector>

#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/map/config.hpp"

namespace mss::map::testing {

struct RefNeighbor {
  float dist2;
  std::array<float, 3> p;
};

class RefMap {
 public:
  using Key = std::array<std::int32_t, 3>;

  explicit RefMap(const MapConfig& cfg) : cfg_(cfg) {}

  void insert(const common::ScanFrame& scan, const common::Pose& T_w_l) {
    const common::Span<const float> xyz = scan.xyz();
    for (std::size_t i = 0; i < scan.size(); ++i) {
      const common::Vec3 p_l(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
      const common::Vec3 p_w = T_w_l * p_l;
      const std::array<float, 3> pf{
          static_cast<float>(p_w.x()), static_cast<float>(p_w.y()), static_cast<float>(p_w.z())};
      const std::optional<Key> key = key_of(pf);
      if (!key) {
        continue;
      }
      auto it = voxels_.find(*key);
      if (it == voxels_.end()) {
        if (voxels_.size() == cfg_.max_voxels) {
          ++dropped_;
          continue;
        }
        Ring ring;
        ring.slots.resize(static_cast<std::size_t>(cfg_.max_points_per_voxel));
        it = voxels_.emplace(*key, ring).first;
      }
      Ring& r = it->second;
      r.slots[static_cast<std::size_t>(r.head)] = pf;
      r.head = (r.head + 1) % cfg_.max_points_per_voxel;
      r.count = std::min(r.count + 1, cfg_.max_points_per_voxel);
    }
  }

  // Scans ALL stored points and keeps those whose voxel key is within 1 of the query key on
  // every axis. Stable-sorts them by (double d2, visit index (dz+1)*9+(dy+1)*3+(dx+1), slot)
  // and returns the first min(k_eff, 32).
  std::vector<RefNeighbor> knn(const common::Vec3& p_w, int k) const;

  void prune(const common::Vec3& p_w_b) {
    const double r2 = cfg_.prune_radius_m * cfg_.prune_radius_m;
    for (auto it = voxels_.begin(); it != voxels_.end();) {
      const common::Vec3 c((it->first[0] + 0.5) * cfg_.voxel_m,
                           (it->first[1] + 0.5) * cfg_.voxel_m,
                           (it->first[2] + 0.5) * cfg_.voxel_m);
      if ((c - p_w_b).squaredNorm() > r2) {
        it = voxels_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::size_t num_voxels() const { return voxels_.size(); }
  std::size_t num_points() const {
    std::size_t n = 0;
    for (const auto& kv : voxels_) {
      n += static_cast<std::size_t>(kv.second.count);
    }
    return n;
  }
  std::size_t num_dropped() const { return dropped_; }

 private:
  struct Ring {
    std::vector<std::array<float, 3>> slots;
    int count = 0;
    int head = 0;
  };

  std::optional<Key> key_of_double(const common::Vec3& p) const {
    Key key{};
    for (int a = 0; a < 3; ++a) {
      const double f = std::floor(p[a] / cfg_.voxel_m);
      if (!(f >= std::numeric_limits<std::int32_t>::min() + 1.0 &&
            f <= std::numeric_limits<std::int32_t>::max() - 1.0)) {
        return std::nullopt;
      }
      key[static_cast<std::size_t>(a)] = static_cast<std::int32_t>(f);
    }
    return key;
  }
  std::optional<Key> key_of(const std::array<float, 3>& p) const {
    return key_of_double(common::Vec3(p[0], p[1], p[2]));
  }

  MapConfig cfg_;
  std::map<Key, Ring> voxels_;
  std::size_t dropped_ = 0;
};

// Values from config/default.yaml (map.*).
inline MapConfig default_map_config() {
  MapConfig c;
  c.voxel_m = 0.5;
  c.max_points_per_voxel = 20;
  c.knn_k = 5;
  c.prune_radius_m = 100.0;
  c.prune_period_s = 5.0;
  c.max_voxels = 500000;
  return c;
}

// Appends `pts` to `scan` (cast to float). The scan must have room for them.
inline void push_points(common::ScanFrame& scan, const std::vector<common::Vec3>& pts) {
  for (const common::Vec3& p : pts) {
    scan.push(static_cast<float>(p.x()),
              static_cast<float>(p.y()),
              static_cast<float>(p.z()),
              0.0F,
              0U,
              0U);
  }
}

}  // namespace mss::map::testing
