#include "mss/io/pcd_writer.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <system_error>

namespace mss::io {
namespace {

static_assert(sizeof(Eigen::Vector3f) == 3 * sizeof(float),
              "Eigen::Vector3f must be three packed floats (PCD record layout)");

struct FileCloser {
  void operator()(std::FILE* f) const {
    if (f != nullptr) {
      std::fclose(f);
    }
  }
};
using FilePtr = std::unique_ptr<std::FILE, FileCloser>;

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
  FilePtr f(std::fopen(path.c_str(), "wb"));
  if (!f) {
    return common::Status::error("write_pcd: cannot open " + path.string());
  }
  const std::size_t n = points.size();
  const int header_ok = std::fprintf(f.get(),
                                     "# .PCD v0.7 - Point Cloud Data file format\n"
                                     "VERSION 0.7\n"
                                     "FIELDS x y z\n"
                                     "SIZE 4 4 4\n"
                                     "TYPE F F F\n"
                                     "COUNT 1 1 1\n"
                                     "WIDTH %zu\n"
                                     "HEIGHT 1\n"
                                     "VIEWPOINT 0 0 0 1 0 0 0\n"
                                     "POINTS %zu\n"
                                     "DATA binary\n",
                                     n,
                                     n);
  if (header_ok < 0) {
    return common::Status::error("write_pcd: cannot write " + path.string());
  }
  if (n > 0 && std::fwrite(points.data(), sizeof(Eigen::Vector3f), n, f.get()) != n) {
    return common::Status::error("write_pcd: cannot write " + path.string());
  }
  if (std::fclose(f.release()) != 0) {
    return common::Status::error("write_pcd: cannot close " + path.string());
  }
  return common::Status::ok();
}

}  // namespace mss::io
