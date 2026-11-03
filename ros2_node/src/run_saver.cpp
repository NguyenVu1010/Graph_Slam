#include "mss_ros2/run_saver.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>

#include <Eigen/Geometry>

#include "mss/common/span.hpp"
#include "mss/io/pcd_writer.hpp"

namespace mss_ros2 {

namespace {

namespace fs = std::filesystem;

// Seconds from integer ns, 9 decimals, as mss_bench writes TUM time.
std::string seconds_text(mss::common::Timestamp t) {
  const std::int64_t ns = t.ns;
  const bool neg = ns < 0;
  const std::uint64_t mag =
      neg ? static_cast<std::uint64_t>(-(ns + 1)) + 1U : static_cast<std::uint64_t>(ns);
  const auto per_s = static_cast<std::uint64_t>(mss::common::kNanosecondsPerSecond);
  char buf[48];
  std::snprintf(
      buf, sizeof(buf), "%s%" PRIu64 ".%09" PRIu64, neg ? "-" : "", mag / per_s, mag % per_s);
  return buf;
}

const char* state_text(mss::pipeline::LoopState s) {
  switch (s) {
    case mss::pipeline::LoopState::kActive:
      return "active";
    case mss::pipeline::LoopState::kReleased:
      return "released";
    case mss::pipeline::LoopState::kRemoved:
      return "removed";
  }
  return "unknown";
}

constexpr const char* kKfOptHeader =
    "# kf_opt.tum from /mss/save: every keyframe of the backend graph at its optimised T_w_b\n"
    "# (mss_bench's kf_opt.tum lists only the keyframes still in the map store)\n"
    "# t x y z qx qy qz qw\n";

// Writes `text` to `<path>.tmp` and renames it over `path`.
mss::common::Status write_text(const fs::path& path, const std::string& text) {
  const fs::path tmp = fs::path(path.string() + ".tmp");
  {
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (!f) {
      return mss::common::Status::error("cannot open " + tmp.string());
    }
    f << text;
    f.flush();
    if (!f) {
      std::error_code ec;
      fs::remove(tmp, ec);
      return mss::common::Status::error("cannot write " + tmp.string());
    }
  }
  std::error_code ec;
  fs::rename(tmp, path, ec);
  if (ec) {
    fs::remove(tmp, ec);
    return mss::common::Status::error("cannot rename " + tmp.string() + " to " + path.string());
  }
  return mss::common::Status::ok();
}

}  // namespace

std::string tum_line(mss::common::Timestamp t, const mss::common::Pose& T_w_b) {
  // Hamilton quaternion of R_w_b; Eigen stores (x, y, z, w), TUM writes (qx qy qz qw).
  const Eigen::Quaterniond q(T_w_b.R);
  char buf[256];
  std::snprintf(buf,
                sizeof(buf),
                " %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
                T_w_b.p.x(),
                T_w_b.p.y(),
                T_w_b.p.z(),
                q.x(),
                q.y(),
                q.z(),
                q.w());
  return seconds_text(t) + buf;
}

std::string loops_csv(const mss::pipeline::GraphSnapshot& snap) {
  std::string out = "i,j,t_i_s,t_j_s,state\n";
  for (const mss::pipeline::GraphLoop& l : snap.loops) {
    out += std::to_string(l.i) + "," + std::to_string(l.j) + "," + seconds_text(l.t_i) + "," +
           seconds_text(l.t_j) + "," + state_text(l.state) + "\n";
  }
  return out;
}

mss::common::Result<std::pair<fs::path, int>> new_save_dir(const fs::path& run_dir, int first_n) {
  using R = mss::common::Result<std::pair<fs::path, int>>;
  std::error_code ec;
  fs::create_directories(run_dir, ec);
  if (ec || !fs::is_directory(run_dir)) {
    return R::error("cannot create directory " + run_dir.string());
  }
  for (int n = std::max(first_n, 1); n < std::max(first_n, 1) + kMaxSaveDirs; ++n) {
    char name[32];
    std::snprintf(name, sizeof(name), "save-%03d", n);
    const fs::path d = run_dir / name;
    if (fs::create_directory(d, ec) && !ec) {  // true only when this call created it
      return R::ok({d, n});
    }
    // Taken: an existing directory, or a regular file or other non-directory of that name
    // (create_directory then reports an error), is skipped (review N3-2). Any other error
    // (permissions, disk) stops the search.
    std::error_code ec_exists;
    if (ec && !fs::exists(fs::symlink_status(d, ec_exists))) {
      return R::error("cannot create directory " + d.string());
    }
  }
  return R::error("no free save-<n> directory in " + run_dir.string());
}

mss::common::Result<SaveSummary> save_run(const fs::path& dir,
                                          const mss::pipeline::GraphSnapshot& snap,
                                          const std::vector<Eigen::Vector3f>& map_w) {
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec || !fs::is_directory(dir)) {
    return mss::common::Result<SaveSummary>::error("cannot create directory " + dir.string());
  }
  // A comment header (TUM readers skip '#' lines) so this file is not mistaken for mss_bench's
  // kf_opt.tum, which lists only the keyframes still in the map store (review N-8).
  std::string tum = kKfOptHeader;
  for (const mss::pipeline::GraphKeyframe& k : snap.keyframes) {
    tum += tum_line(k.t, k.T_w_b);
  }
  mss::common::Status st = write_text(dir / "kf_opt.tum", tum);
  if (st.is_ok()) {
    st = write_text(dir / "loops.csv", loops_csv(snap));
  }
  if (st.is_ok()) {
    st = mss::io::write_pcd(dir / "map.pcd", mss::common::Span<const Eigen::Vector3f>(map_w));
  }
  if (!st.is_ok()) {
    return mss::common::Result<SaveSummary>::error(st.reason());
  }
  SaveSummary s;
  s.dir = dir;
  s.n_keyframes = snap.keyframes.size();
  s.n_loops = snap.loops.size();
  s.n_points = map_w.size();
  return mss::common::Result<SaveSummary>::ok(s);
}

}  // namespace mss_ros2
