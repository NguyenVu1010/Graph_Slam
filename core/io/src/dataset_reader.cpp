#include "mss/io/dataset_reader.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <limits>
#include <string>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace mss::io {

bool host_is_little_endian() {
  const std::uint16_t probe = 0x0102U;
  unsigned char first = 0;
  std::memcpy(&first, &probe, 1);
  return first == 0x02U;
}

namespace {

using common::Result;
using common::Timestamp;

// Records read from disk per read(2) call for each record stream. Fixed I/O chunk, not a
// tunable: it only trades syscalls for memory (at most 1024 * 105 B per stream).
constexpr std::size_t kStreamChunkRecords = 1024;
// Room for "/" + digits of a size_t (at most 20) + ".bin" + NUL after the lidar dir.
constexpr std::size_t kLidarNameRoom = 32;

// read(2) until `n` bytes or EOF. Returns the bytes read, or -1 on an I/O error.
long read_full(int fd, void* dst, std::size_t n) {
  auto* p = static_cast<unsigned char*>(dst);
  std::size_t got = 0;
  while (got < n) {
    const ssize_t r = ::read(fd, p + got, n - got);
    if (r < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    }
    if (r == 0) {
      break;
    }
    got += static_cast<std::size_t>(r);
  }
  return static_cast<long>(got);
}

class Fd {
 public:
  Fd() = default;
  explicit Fd(int fd) : fd_(fd) {}
  Fd(const Fd&) = delete;
  Fd& operator=(const Fd&) = delete;
  Fd(Fd&& o) noexcept : fd_(o.fd_) { o.fd_ = -1; }
  Fd& operator=(Fd&& o) noexcept {
    if (this != &o) {
      reset();
      fd_ = o.fd_;
      o.fd_ = -1;
    }
    return *this;
  }
  ~Fd() { reset(); }
  int get() const { return fd_; }
  bool valid() const { return fd_ >= 0; }
  void reset(int fd = -1) {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = fd;
  }

 private:
  int fd_ = -1;
};

// A fixed-size-record file read in chunks into a buffer allocated once.
struct RecordStream {
  Fd fd;
  std::string path;            // for error messages
  std::size_t rec_size = 0;    // B
  std::size_t unread = 0;      // records still in the file
  std::size_t next_index = 0;  // index of the next record handed out
  std::vector<unsigned char> buf;
  std::size_t pos = 0;  // B consumed in buf
  std::size_t len = 0;  // B valid in buf
  bool has_head = false;
  std::int64_t last_t = std::numeric_limits<std::int64_t>::min();

  // Copies the next raw record into `dst`. Returns false with `err` set on a short read.
  bool pop(void* dst, std::string& err) {
    if (pos == len) {
      const std::size_t n = std::min(unread, kStreamChunkRecords);
      const long got = read_full(fd.get(), buf.data(), n * rec_size);
      if (got < 0 || static_cast<std::size_t>(got) != n * rec_size) {
        err = path + ": short read at record " + std::to_string(next_index);
        return false;
      }
      unread -= n;
      pos = 0;
      len = n * rec_size;
    }
    std::memcpy(dst, buf.data() + pos, rec_size);
    pos += rec_size;
    ++next_index;
    return true;
  }

  // Checks that t does not go backwards within this stream.
  bool check_t(std::int64_t t, std::string& err) {
    if (t < last_t) {
      err = path + ": record " + std::to_string(next_index - 1) + " has t " + std::to_string(t) +
            " ns before the previous " + std::to_string(last_t) + " ns";
      return false;
    }
    last_t = t;
    return true;
  }
};

common::Mat3 rotation_from_wxyz(double w, double x, double y, double z) {
  // Eigen::Quaterniond takes Hamilton (w, x, y, z) in its constructor. Normalised because
  // the file stores doubles that may be slightly off unit length.
  const Eigen::Quaterniond q = Eigen::Quaterniond(w, x, y, z).normalized();
  return q.toRotationMatrix();
}

// ---- index.json parsing (yaml-cpp: JSON is valid YAML) ----

bool read_vec(const YAML::Node& node, std::size_t n, double* out) {
  if (!node || !node.IsSequence() || node.size() != n) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = node[i].as<double>();
    if (!std::isfinite(out[i])) {
      return false;
    }
  }
  return true;
}

struct TopicFiles {
  std::string imu, gnss, odom, gt, lidar_dir;
};

// Throws YAML exceptions on wrong scalar types; the caller catches them.
std::string parse_index(const YAML::Node& root, DatasetIndex& idx, TopicFiles& files) {
  if (!root.IsMap()) {
    return "top level is not a mapping";
  }
  if (!root["version"]) {
    return "missing key 'version'";
  }
  idx.version = root["version"].as<int>();
  if (idx.version != kFormatVersion) {
    return "version " + std::to_string(idx.version) + " is not supported (expected " +
           std::to_string(kFormatVersion) + ")";
  }
  const YAML::Node topics = root["topics"];
  if (!topics || !topics.IsMap()) {
    return "missing mapping 'topics'";
  }
  const auto topic =
      [&topics](
          const char* name, const char* key, std::string& file, std::size_t& count) -> std::string {
    const YAML::Node t = topics[name];
    if (!t || !t.IsMap() || !t[key] || !t["count"]) {
      return std::string("topics.") + name + " needs '" + key + "' and 'count'";
    }
    file = t[key].as<std::string>();
    const long long c = t["count"].as<long long>();
    if (c < 0) {
      return std::string("topics.") + name + ".count is negative";
    }
    count = static_cast<std::size_t>(c);
    return {};
  };
  for (const auto& r : {topic("imu", "file", files.imu, idx.n_imu),
                        topic("gnss", "file", files.gnss, idx.n_gnss),
                        topic("odom", "file", files.odom, idx.n_odom),
                        topic("gt", "file", files.gt, idx.n_gt),
                        topic("lidar", "dir", files.lidar_dir, idx.n_lidar)}) {
    if (!r.empty()) {
      return r;
    }
  }
  double lla[3];
  if (!read_vec(root["origin_lla"], 3, lla)) {
    return "'origin_lla' must be 3 finite numbers [lat_deg, lon_deg, alt_m]";
  }
  idx.origin_lla = gnss::Lla{lla[0], lla[1], lla[2]};
  const YAML::Node tbl = root["T_b_l"];
  double t[3];
  double q[4];  // Hamilton (w, x, y, z)
  if (!tbl || !tbl.IsMap() || !read_vec(tbl["translation_m"], 3, t) ||
      !read_vec(tbl["quaternion_wxyz"], 4, q)) {
    return "'T_b_l' needs translation_m (3 numbers) and quaternion_wxyz (4 numbers)";
  }
  const double qn = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!(qn > 0.0)) {
    return "'T_b_l.quaternion_wxyz' has zero norm";
  }
  idx.T_b_l.p = common::Vec3(t[0], t[1], t[2]);
  idx.T_b_l.R = rotation_from_wxyz(q[0], q[1], q[2], q[3]);
  double pbg[3];
  if (!read_vec(root["p_b_g_m"], 3, pbg)) {
    return "'p_b_g_m' must be 3 finite numbers";
  }
  idx.p_b_g = common::Vec3(pbg[0], pbg[1], pbg[2]);
  if (!root["body_frame"] || !root["lidar_frame"] || !root["gnss_cov_filled_from_sigma"]) {
    return "missing 'body_frame', 'lidar_frame' or 'gnss_cov_filled_from_sigma'";
  }
  idx.body_frame = root["body_frame"].as<std::string>();
  idx.lidar_frame = root["lidar_frame"].as<std::string>();
  idx.gnss_cov_filled_from_sigma = root["gnss_cov_filled_from_sigma"].as<bool>();
  return {};
}

}  // namespace

struct DatasetReader::Impl {
  DatasetIndex index;
  std::size_t max_points = 0;

  RecordStream imu, gnss, odom, gt;
  common::ImuSample imu_head;
  common::GnssMeasurement gnss_head;
  common::WheelSample odom_head;
  common::NavState gt_head;

  // Lidar: the next scan's file is opened and its header validated before it is merged.
  std::vector<char> lidar_path;  // "<dir>/<lidar_dir>/" followed by the file name
  std::size_t lidar_prefix_len = 0;
  std::size_t lidar_next = 0;  // index of the next scan file to peek
  bool lidar_has_head = false;
  Fd lidar_fd;
  LidarHeader lidar_header{};
  std::int64_t lidar_last_t = std::numeric_limits<std::int64_t>::min();
  std::vector<LidarPoint> staging;  // max_points AoS points, allocated in open()

  std::string error;  // sticky; empty while healthy

  const char* set_lidar_path(std::size_t k) {
    std::snprintf(lidar_path.data() + lidar_prefix_len,
                  kLidarNameRoom,
                  "%0*zu.bin",
                  static_cast<int>(kLidarNameWidth),
                  k);
    return lidar_path.data();
  }

  bool fail(std::string reason) {
    error = std::move(reason);
    return false;
  }

  bool fill_imu() {
    ImuRecord r;
    if (!imu.pop(&r, error) || !imu.check_t(r.t, error)) {
      return false;
    }
    imu_head.t = Timestamp{r.t};
    imu_head.acc_b = common::Vec3(r.acc[0], r.acc[1], r.acc[2]);
    imu_head.gyro_b = common::Vec3(r.gyro[0], r.gyro[1], r.gyro[2]);
    imu.has_head = true;
    return true;
  }

  bool fill_gnss() {
    GnssRecord r;
    if (!gnss.pop(&r, error) || !gnss.check_t(r.t, error)) {
      return false;
    }
    gnss_head.t = Timestamp{r.t};
    gnss_head.p_e_g = common::Vec3(r.p_e_g[0], r.p_e_g[1], r.p_e_g[2]);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        gnss_head.cov(i, j) = r.cov[3 * i + j];  // row-major on disk
      }
    }
    gnss_head.status = r.status;
    gnss.has_head = true;
    return true;
  }

  bool fill_odom() {
    OdomRecord r;
    if (!odom.pop(&r, error) || !odom.check_t(r.t, error)) {
      return false;
    }
    odom_head.t = Timestamp{r.t};
    odom_head.v_mps = r.v;
    odom_head.omega_radps = r.omega;
    odom.has_head = true;
    return true;
  }

  bool fill_gt() {
    GtRecord r;
    if (!gt.pop(&r, error) || !gt.check_t(r.t, error)) {
      return false;
    }
    gt_head.t = Timestamp{r.t};
    gt_head.T_w_b.p = common::Vec3(r.p[0], r.p[1], r.p[2]);
    // q_wxyz is Hamilton (w, x, y, z).
    gt_head.T_w_b.R = rotation_from_wxyz(r.q_wxyz[0], r.q_wxyz[1], r.q_wxyz[2], r.q_wxyz[3]);
    gt_head.v_w_b.setZero();
    gt_head.bias = common::ImuBias{};
    gt.has_head = true;
    return true;
  }

  // Opens the next scan file and validates its header and size against n.
  bool fill_lidar() {
    const char* path = set_lidar_path(lidar_next);
    lidar_fd.reset(::open(path, O_RDONLY | O_CLOEXEC));
    if (!lidar_fd.valid()) {
      return fail(std::string(path) + ": cannot open: " + std::strerror(errno));
    }
    struct stat st {};
    if (::fstat(lidar_fd.get(), &st) != 0) {
      return fail(std::string(path) + ": fstat failed: " + std::strerror(errno));
    }
    const auto size = static_cast<std::uint64_t>(st.st_size);
    if (size < sizeof(LidarHeader)) {
      return fail(std::string(path) + ": " + std::to_string(size) +
                  " B is shorter than the 12 B header");
    }
    if (read_full(lidar_fd.get(), &lidar_header, sizeof(LidarHeader)) !=
        static_cast<long>(sizeof(LidarHeader))) {
      return fail(std::string(path) + ": short read of the header");
    }
    const std::uint32_t n = lidar_header.n;
    if (n > max_points) {
      return fail(std::string(path) + ": " + std::to_string(n) + " points > max_points_per_scan " +
                  std::to_string(max_points));
    }
    const std::uint64_t want = sizeof(LidarHeader) + std::uint64_t{n} * sizeof(LidarPoint);
    if (size != want) {
      return fail(std::string(path) + ": size " + std::to_string(size) + " B, expected 12 + " +
                  std::to_string(n) + " * 22 = " + std::to_string(want) + " B");
    }
    if (lidar_header.t_start < lidar_last_t) {
      return fail(std::string(path) + ": t_start " + std::to_string(lidar_header.t_start) +
                  " ns is before the previous scan's " + std::to_string(lidar_last_t) + " ns");
    }
    lidar_last_t = lidar_header.t_start;
    ++lidar_next;
    lidar_has_head = true;
    return true;
  }

  // Reads the peeked scan's points into `scan` (SoA) and closes the file.
  bool emit_lidar(common::ScanFrame& scan) {
    const std::size_t n = lidar_header.n;
    const char* path = lidar_path.data();  // still names the peeked file
    if (scan.capacity() < n) {
      return fail(std::string(path) + ": " + std::to_string(n) +
                  " points exceed the ScanFrame capacity " + std::to_string(scan.capacity()));
    }
    const std::size_t bytes = n * sizeof(LidarPoint);
    if (n > 0 && read_full(lidar_fd.get(), staging.data(), bytes) != static_cast<long>(bytes)) {
      return fail(std::string(path) + ": short read of the points");
    }
    lidar_fd.reset();
    scan.clear();
    float* xyz = scan.xyz_mut();
    float* intensity = scan.intensity_mut();
    std::uint32_t* t_off = scan.t_offset_ns_mut();
    std::uint16_t* ring = scan.ring_mut();
    for (std::size_t i = 0; i < n; ++i) {
      const LidarPoint& p = staging[i];
      xyz[3 * i] = p.xyz[0];
      xyz[3 * i + 1] = p.xyz[1];
      xyz[3 * i + 2] = p.xyz[2];
      intensity[i] = p.intensity;
      t_off[i] = p.t_offset_ns;
      ring[i] = p.ring;
    }
    scan.set_size(n);
    scan.t_start = Timestamp{lidar_header.t_start};
    lidar_has_head = false;
    return true;
  }

  // Makes sure every non-exhausted stream has a head record.
  bool refill() {
    if (!imu.has_head && imu.next_index < index.n_imu && !fill_imu()) {
      return false;
    }
    if (!gnss.has_head && gnss.next_index < index.n_gnss && !fill_gnss()) {
      return false;
    }
    if (!odom.has_head && odom.next_index < index.n_odom && !fill_odom()) {
      return false;
    }
    if (!gt.has_head && gt.next_index < index.n_gt && !fill_gt()) {
      return false;
    }
    if (!lidar_has_head && lidar_next < index.n_lidar && !fill_lidar()) {
      return false;
    }
    return true;
  }
};

DatasetReader::DatasetReader(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
DatasetReader::DatasetReader(DatasetReader&& other) noexcept = default;
DatasetReader& DatasetReader::operator=(DatasetReader&& other) noexcept = default;
DatasetReader::~DatasetReader() = default;

const DatasetIndex& DatasetReader::index() const {
  return impl_->index;
}

Result<DatasetReader> DatasetReader::open(const std::filesystem::path& dir,
                                          std::size_t max_points_per_scan) {
  using R = Result<DatasetReader>;
  if (!host_is_little_endian()) {
    return R::error("DatasetReader: mssdata is little-endian; big-endian hosts are unsupported");
  }
  auto impl = std::make_unique<Impl>();
  impl->max_points = max_points_per_scan;

  const std::filesystem::path index_path = dir / "index.json";
  std::error_code ec;
  if (!std::filesystem::is_regular_file(index_path, ec)) {
    return R::error(index_path.string() + ": missing");
  }
  TopicFiles files;
  try {
    const YAML::Node root = YAML::LoadFile(index_path.string());
    const std::string problem = parse_index(root, impl->index, files);
    if (!problem.empty()) {
      return R::error(index_path.string() + ": " + problem);
    }
  } catch (const std::exception& e) {
    return R::error(index_path.string() + ": cannot parse: " + e.what());
  }

  const auto open_stream = [&dir](RecordStream& s,
                                  const std::string& name,
                                  std::size_t rec_size,
                                  std::size_t count) -> std::string {
    const std::filesystem::path p = dir / name;
    s.path = p.string();
    s.rec_size = rec_size;
    std::error_code fec;
    const std::uintmax_t size = std::filesystem::file_size(p, fec);
    if (fec) {
      return s.path + ": missing or unreadable: " + fec.message();
    }
    if (size % rec_size != 0) {
      return s.path + ": size " + std::to_string(size) + " B is not a multiple of the " +
             std::to_string(rec_size) + " B record";
    }
    if (size / rec_size != count) {
      return s.path + ": holds " + std::to_string(size / rec_size) +
             " records but index.json says " + std::to_string(count);
    }
    s.fd.reset(::open(p.c_str(), O_RDONLY | O_CLOEXEC));
    if (!s.fd.valid()) {
      return s.path + ": cannot open: " + std::strerror(errno);
    }
    s.unread = count;
    s.buf.resize(std::min(count, kStreamChunkRecords) * rec_size);
    return {};
  };
  const DatasetIndex& idx = impl->index;
  for (const std::string& problem :
       {open_stream(impl->imu, files.imu, sizeof(ImuRecord), idx.n_imu),
        open_stream(impl->gnss, files.gnss, sizeof(GnssRecord), idx.n_gnss),
        open_stream(impl->odom, files.odom, sizeof(OdomRecord), idx.n_odom),
        open_stream(impl->gt, files.gt, sizeof(GtRecord), idx.n_gt)}) {
    if (!problem.empty()) {
      return R::error(problem);
    }
  }

  const std::filesystem::path lidar_dir = dir / files.lidar_dir;
  if (idx.n_lidar > 0) {
    if (!std::filesystem::is_directory(lidar_dir, ec)) {
      return R::error(lidar_dir.string() + ": missing directory");
    }
    char name[kLidarNameRoom];
    for (std::size_t k = 0; k < idx.n_lidar; ++k) {
      std::snprintf(name, sizeof(name), "%0*zu.bin", static_cast<int>(kLidarNameWidth), k);
      if (!std::filesystem::is_regular_file(lidar_dir / name, ec)) {
        return R::error((lidar_dir / name).string() + ": missing (index.json lists " +
                        std::to_string(idx.n_lidar) + " scans)");
      }
    }
  }
  const std::string prefix = lidar_dir.string() + "/";
  impl->lidar_prefix_len = prefix.size();
  impl->lidar_path.assign(prefix.size() + kLidarNameRoom, '\0');
  std::memcpy(impl->lidar_path.data(), prefix.data(), prefix.size());
  impl->staging.resize(max_points_per_scan);
  return R::ok(DatasetReader(std::move(impl)));
}

Result<bool> DatasetReader::next(Record& out, common::ScanFrame& scan) {
  Impl& d = *impl_;
  if (!d.error.empty() || !d.refill()) {
    return Result<bool>::error(d.error);
  }
  // Pick the smallest t; strict < keeps the tie order imu, odom, gnss, gt, lidar.
  RecordKind kind = RecordKind::kImu;
  bool found = false;
  std::int64_t best = 0;
  const auto consider = [&](bool has, std::int64_t t, RecordKind k) {
    if (has && (!found || t < best)) {
      found = true;
      best = t;
      kind = k;
    }
  };
  consider(d.imu.has_head, d.imu_head.t.ns, RecordKind::kImu);
  consider(d.odom.has_head, d.odom_head.t.ns, RecordKind::kOdom);
  consider(d.gnss.has_head, d.gnss_head.t.ns, RecordKind::kGnss);
  consider(d.gt.has_head, d.gt_head.t.ns, RecordKind::kGt);
  consider(d.lidar_has_head, d.lidar_header.t_start, RecordKind::kLidar);
  if (!found) {
    return Result<bool>::ok(false);
  }

  out.kind = kind;
  out.t = Timestamp{best};
  switch (kind) {
    case RecordKind::kImu:
      out.imu = d.imu_head;
      d.imu.has_head = false;
      break;
    case RecordKind::kOdom:
      out.odom = d.odom_head;
      d.odom.has_head = false;
      break;
    case RecordKind::kGnss:
      out.gnss = d.gnss_head;
      d.gnss.has_head = false;
      break;
    case RecordKind::kGt:
      out.gt = d.gt_head;
      d.gt.has_head = false;
      break;
    case RecordKind::kLidar:
      if (!d.emit_lidar(scan)) {
        return Result<bool>::error(d.error);
      }
      break;
  }
  return Result<bool>::ok(true);
}

}  // namespace mss::io
