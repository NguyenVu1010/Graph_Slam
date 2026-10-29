#include "mss/io/pcd_writer.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <ostream>
#include <string>
#include <system_error>

namespace mss::io {
namespace {

static_assert(sizeof(Eigen::Vector3f) == 3 * sizeof(float),
              "Eigen::Vector3f must be three packed floats (PCD record layout)");

bool host_is_little_endian() {
  const std::uint32_t one = 1U;
  unsigned char first = 0;
  std::memcpy(&first, &one, 1);
  return first == 1U;
}

}  // namespace

common::Status write_pcd(const std::filesystem::path& path,
                         common::Span<const Eigen::Vector3f> points) {
  namespace fs = std::filesystem;
  if (path.empty()) {
    return common::Status::error("write_pcd: empty path");
  }
  std::error_code ec;
  if (fs::is_directory(path, ec)) {
    return common::Status::error("write_pcd: " + path.string() + " is a directory");
  }
  const fs::path parent = path.parent_path();
  if (!parent.empty() && !fs::is_directory(parent, ec)) {
    return common::Status::error("write_pcd: parent directory of " + path.string() +
                                 " does not exist");
  }
  if (!host_is_little_endian()) {
    return common::Status::error("write_pcd: big-endian host is not supported");
  }
  // Write next to the target, then rename: a failed or interrupted write never leaves a
  // truncated `path` (the previous file, if any, stays). std::ofstream takes the path in its
  // native encoding, so this also opens on Windows.
  const fs::path tmp = path.string() + ".tmp";
  const std::size_t n = points.size();
  {
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (!f) {
      return common::Status::error("write_pcd: cannot open " + tmp.string());
    }
    f << "# .PCD v0.7 - Point Cloud Data file format\n"
         "VERSION 0.7\n"
         "FIELDS x y z\n"
         "SIZE 4 4 4\n"
         "TYPE F F F\n"
         "COUNT 1 1 1\n"
         "WIDTH "
      << n << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << n << "\nDATA binary\n";
    if (n > 0) {
      f.write(reinterpret_cast<const char*>(points.data()),
              static_cast<std::streamsize>(n * sizeof(Eigen::Vector3f)));
    }
    f.close();
    if (f.fail()) {
      std::error_code rm_ec;
      fs::remove(tmp, rm_ec);
      return common::Status::error("write_pcd: cannot write " + path.string());
    }
  }
  fs::rename(tmp, path, ec);
  if (ec) {
    std::error_code rm_ec;
    fs::remove(tmp, rm_ec);
    return common::Status::error("write_pcd: cannot replace " + path.string() + ": " +
                                 ec.message());
  }
  return common::Status::ok();
}

}  // namespace mss::io
