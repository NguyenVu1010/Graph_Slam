#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "mss/common/result.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/pipeline/graph_snapshot.hpp"

// Files written by the /mss/save service (viz.enabled, ADR-031). No ROS state: pure functions,
// tested in test/test_run_saver.cpp. Off the hot path (on request).
namespace mss_ros2 {

/// What save_run() wrote.
struct SaveSummary {
  std::filesystem::path dir;    ///< the run directory the three files are in
  std::size_t n_keyframes = 0;  ///< lines of kf_opt.tum
  std::size_t n_loops = 0;      ///< data rows of loops.csv
  std::size_t n_points = 0;     ///< points of map.pcd
};

/// One TUM line "t x y z qx qy qz qw\n": t in seconds printed from the integer nanoseconds (9
/// decimals, never through double), p_w_b in m, the Hamilton quaternion of R_w_b written in TUM
/// order (qx qy qz qw). Same format as mss_bench's est.tum and kf_opt.tum.
std::string tum_line(mss::common::Timestamp t, const mss::common::Pose& T_w_b);

/// loops.csv: header "i,j,t_i_s,t_j_s,state", then one row per loop of `snap` in snapshot order;
/// t in seconds from integer ns (9 decimals); state "active", "released" or "removed".
std::string loops_csv(const mss::pipeline::GraphSnapshot& snap);

/// Writes into `dir` (created with its parents if missing):
///   - kf_opt.tum: every keyframe of `snap` (all the backend has, including those evicted from
///     the map store; mss_bench's kf_opt.tum lists only the stored ones) at its optimised T_w_b,
///     after three '#' comment lines that say so (TUM readers skip them);
///   - loops.csv (loops_csv);
///   - map.pcd: `map_w` (Estimator::export_map, points in w, m) through io::write_pcd.
/// Each file is written to `<name>.tmp` and renamed over `<name>`, so a file is either the
/// previous one or complete. Empty inputs write valid empty files (header only).
/// Errors (nothing thrown): Status error naming the file when the directory cannot be created or
///   a file cannot be written or renamed; files written before the failure stay.
/// Save directories tried by new_save_dir() before it gives up.
constexpr int kMaxSaveDirs = 10000;

/// Creates a new, empty `<run_dir>/save-<n>` (n zero-padded to 3 digits) for the first n >=
/// `first_n` whose name is not taken yet (a directory, a regular file or anything else of that
/// name is skipped, review N3-2), creating `run_dir` and its parents if needed.
/// An existing directory is never reused, also after a restart with the same run id or by
/// another node writing the same run directory (review N2-3): the creation itself
/// (non-recursive create_directory) decides. Returns the directory and its n.
/// Errors: Status error naming the directory when `run_dir` cannot be created, or when no free
///   n is found within kMaxSaveDirs tries.
mss::common::Result<std::pair<std::filesystem::path, int>> new_save_dir(
    const std::filesystem::path& run_dir, int first_n);

mss::common::Result<SaveSummary> save_run(const std::filesystem::path& dir,
                                          const mss::pipeline::GraphSnapshot& snap,
                                          const std::vector<Eigen::Vector3f>& map_w);

}  // namespace mss_ros2
