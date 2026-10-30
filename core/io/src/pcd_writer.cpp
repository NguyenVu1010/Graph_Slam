#include "mss/io/pcd_writer.hpp"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <ostream>
#include <string>
#include <system_error>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#define MSS_PCD_HAVE_FSYNC 1
#endif

namespace mss::io {
namespace {

static_assert(sizeof(Eigen::Vector3f) == 3 * sizeof(float),
              "Eigen::Vector3f must be three packed floats (PCD record layout)");

std::atomic<int> g_fault{static_cast<int>(detail::PcdFault::kNone)};

bool fault(detail::PcdFault f) {
  return g_fault.load() == static_cast<int>(f);
}

// Flushes the file's data to the storage device (POSIX). std::ofstream gives no handle on
// Windows, where the rename is then atomic against a process crash only.
bool sync_file(const std::filesystem::path& p) {
  if (fault(detail::PcdFault::kSyncFail)) {
    return false;
  }
#ifdef MSS_PCD_HAVE_FSYNC
  const int fd = ::open(p.c_str(), O_RDONLY);
  if (fd < 0) {
    return false;
  }
  const bool ok = ::fsync(fd) == 0;
  ::close(fd);
  return ok;
#else
  (void)p;
  return true;
#endif
}

bool host_is_little_endian() {
  const std::uint32_t one = 1U;
  unsigned char first = 0;
  std::memcpy(&first, &one, 1);
  return first == 1U;
}

void remove_quietly(const std::filesystem::path& p) {
  std::error_code ec;
  std::filesystem::remove(p, ec);
}

}  // namespace

namespace detail {
void set_pcd_fault(PcdFault f) {
  g_fault.store(static_cast<int>(f));
}
}  // namespace detail

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
  // Write next to the target, then flush and rename: a failed or interrupted write never leaves
  // a truncated `path` (the previous file, if any, stays). std::ofstream takes the path in its
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
    if (f.fail() || fault(detail::PcdFault::kWriteFail)) {
      remove_quietly(tmp);
      return common::Status::error("write_pcd: cannot write " + path.string());
    }
  }
  if (!sync_file(tmp)) {
    remove_quietly(tmp);
    return common::Status::error("write_pcd: cannot flush " + tmp.string() + " to disk");
  }
  if (fault(detail::PcdFault::kRenameFail)) {
    ec = std::make_error_code(std::errc::io_error);
  } else {
    fs::rename(tmp, path, ec);
  }
  if (ec) {
    remove_quietly(tmp);
    return common::Status::error("write_pcd: cannot replace " + path.string() + ": " +
                                 ec.message());
  }
  return common::Status::ok();
}

}  // namespace mss::io
