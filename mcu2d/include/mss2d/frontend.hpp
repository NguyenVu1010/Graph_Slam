#pragma once

#include <array>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/distance_field.hpp"
#include "mss2d/merger.hpp"
#include "mss2d/messages.hpp"
#include "mss2d/motion.hpp"
#include "mss2d/occupancy_grid.hpp"
#include "mss2d/points2d.hpp"
#include "mss2d/pose2.hpp"
#include "mss2d/scan_matcher.hpp"

namespace mss2d {

enum class FrontEndStatus : std::uint8_t {
  kInitialised = 0,  ///< first scan: the map frame w is the body pose at its end
  kOk,               ///< scan matched
  kDegenerate,       ///< matched; the prediction is kept along the flagged directions
  kPredicted,        ///< no usable match (too few points): prediction only, not inserted
};

/// Front-end output, one per scan, at the scan end time.
struct ScanPose {
  Timestamp t{};  ///< scan end (Scan2d::key())
  Pose2 T_w_b{};  ///< body pose in the map frame
  FrontEndStatus status = FrontEndStatus::kInitialised;
  int n_points = 0;             ///< points the matcher used
  int n_degenerate = 0;         ///< flagged Hessian directions
  float min_eigenvalue = 0.0F;  ///< smallest Hessian eigenvalue (degeneracy margin)
  float cost_m2 = 0.0F;         ///< matcher cost (mean Huber, m^2)
  std::uint32_t submap_id = 0;  ///< submap matched against (first scan: the one started)
  Pose2 T_s_b{};                ///< body pose in that submap's frame
  /// Sum of u u^T over the flagged Hessian directions, u = their translation part rotated
  /// into w (unit length for a pure translation direction); zero when nothing is flagged.
  Sym2f degenerate_xy_w{};
};

/// Receives the front-end output. The argument is valid only during the call.
class PoseSink {
 public:
  virtual void on_pose(const ScanPose& p) = 0;

 protected:
  ~PoseSink() = default;
};

struct FrontEndStats {
  std::uint32_t n_scans = 0;
  std::uint32_t n_ok = 0;
  std::uint32_t n_degenerate = 0;
  std::uint32_t n_predicted = 0;
  std::uint32_t n_not_converged = 0;  ///< matched with MatchStatus::kNotConverged
  std::uint32_t n_submaps_started = 0;
  std::uint32_t n_submaps_finished = 0;
  std::uint32_t n_submaps_recycled = 0;  ///< finished submaps overwritten (pool full)
};

/// Scan-to-submap LiDAR odometry (docs/frontend.md). Fed by the Merger in merge order:
/// gyro and wheel samples go to the motion model; each scan is deskewed to its end time,
/// predicted from the last pose, matched against the older active submap, inserted into
/// both active submaps, and its pose is sent to the PoseSink. Submaps rotate after
/// scans_per_submap scans or max_distance_m of travel; finished ones stay in a fixed pool of
/// kSubmapPoolSize grids (oldest overwritten) for the backend.
///
/// Frames: w map, b body = IMU, l LiDAR (T_b_l from Config::lidar), s submap. No allocation
/// after construction; about 250 KB, give it static storage. Single-threaded (main loop).
class FrontEnd final : public Sink {
 public:
  /// `cfg` must have passed Config::validate(). `out` may be null.
  FrontEnd(const Config& cfg, PoseSink* out);

  void on_imu(const ImuSample2d& s) override;
  void on_wheel(const WheelSample& s) override;
  void on_scan(const Scan2d& s) override;

  bool has_pose() const { return have_pose_; }
  const ScanPose& last_pose() const { return last_; }
  const FrontEndStats& stats() const { return stats_; }
  float gyro_bias() const { return motion_.gyro_bias(); }

  /// Finished submaps, oldest first; valid until the next on_scan().
  int n_finished_submaps() const { return n_finished_; }
  const OccupancyGrid& finished_submap(int i) const;
  /// Id of finished_submap(i).
  std::uint32_t finished_submap_id(int i) const;
  /// The submap scans are matched against (call only after the first scan).
  const OccupancyGrid& matching_submap() const;
  /// Map pose of the active submap `id`; false when `id` is not active.
  bool active_submap_pose(std::uint32_t id, Pose2& T_w_s) const;

  /// Points of the last scan in the LiDAR frame at its end time (after the range filter
  /// and deskew); valid until the next on_scan().
  const PointCloud2d& last_points() const { return pts_; }
  const Pose2& T_b_l() const { return T_b_l_; }

  /// Backend correction (docs/backend.md section 6): moves the map frame, T_w_new_w_old is
  /// applied on the left of the current pose and of every active submap origin, so the scan
  /// to submap geometry is unchanged. Finished submaps are left as they are.
  void apply_correction(const Pose2& T_wnew_wold);

 private:
  struct Slot {
    std::uint32_t id = 0;
    int n_scans = 0;
  };

  void start_submap(const Pose2& T_w_b);
  void insert_and_rotate(const Pose2& T_w_b);
  bool reached(int slot, const Pose2& T_w_b, float fraction) const;

  Config cfg_;
  PoseSink* out_;
  Pose2 T_b_l_;
  Pose2 T_l_b_;
  MotionModel motion_;
  ScanMatcher matcher_;
  DeskewTable deskew_;
  PointCloud2d pts_;
  DistanceField df_;
  std::array<OccupancyGrid, kSubmapPoolSize> grids_;
  std::array<Slot, kSubmapPoolSize> slots_{};
  std::array<bool, kSubmapPoolSize> in_use_{};
  int active_[2]{-1, -1};  ///< slot indices, [0] = older = matched against
  int n_active_ = 0;
  std::array<int, kSubmapPoolSize> finished_{};  ///< slot indices, oldest first
  int n_finished_ = 0;
  std::uint32_t next_id_ = 0;

  bool have_pose_ = false;
  Timestamp t_prev_{};
  Pose2 T_w_b_{};
  ScanPose last_{};
  FrontEndStats stats_{};
};

}  // namespace mss2d
