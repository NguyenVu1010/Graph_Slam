#include "mss/io/dataset_writer.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace mss::io {
namespace {

using common::Result;
using common::Status;

struct FileCloser {
  void operator()(std::FILE* f) const {
    if (f != nullptr) {
      std::fclose(f);
    }
  }
};
using FilePtr = std::unique_ptr<std::FILE, FileCloser>;

// JSON string literal with the escapes RFC 8259 requires.
std::string json_string(const std::string& s) {
  std::string out = "\"";
  for (const char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  out += '"';
  return out;
}

// Shortest-safe round-trip formatting of a finite double (17 significant digits).
std::string json_number(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

std::string json_array(std::initializer_list<double> values) {
  std::string out = "[";
  bool first = true;
  for (const double v : values) {
    if (!first) {
      out += ", ";
    }
    out += json_number(v);
    first = false;
  }
  out += "]";
  return out;
}

bool all_finite(std::initializer_list<double> values) {
  for (const double v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

// Hamilton quaternion of a rotation matrix, as (w, x, y, z), sign fixed to w >= 0.
Eigen::Quaterniond quaternion_of(const common::Mat3& R) {
  Eigen::Quaterniond q(R);
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() = -q.coeffs();
  }
  return q;
}

}  // namespace

struct DatasetWriter::Impl {
  struct Stream {
    FilePtr file;
    std::string path;
    std::size_t count = 0;
    std::int64_t last_t = std::numeric_limits<std::int64_t>::min();
  };

  std::filesystem::path dir;
  DatasetIndex meta;
  Stream imu, gnss, odom, gt;
  std::size_t n_lidar = 0;
  std::int64_t last_lidar_t = std::numeric_limits<std::int64_t>::min();
  std::vector<LidarPoint> staging;  // grows to the largest scan written
  bool finished = false;
  Status finish_status = Status::ok();

  Status check_open(const char* what) const {
    if (finished) {
      return Status::error(std::string("DatasetWriter: ") + what + " after finish()");
    }
    return Status::ok();
  }

  static Status check_time(const Stream& s, std::int64_t t) {
    if (t < s.last_t) {
      return Status::error(s.path + ": t " + std::to_string(t) + " ns is before the previous " +
                           std::to_string(s.last_t) + " ns");
    }
    return Status::ok();
  }

  template <typename Rec>
  static Status append(Stream& s, const Rec& rec) {
    if (std::fwrite(&rec, sizeof(Rec), 1, s.file.get()) != 1) {
      return Status::error(s.path + ": write failed");
    }
    s.last_t = rec.t;
    ++s.count;
    return Status::ok();
  }

  Status write_index() const {
    const Eigen::Quaterniond q = quaternion_of(meta.T_b_l.R);
    std::string j = "{\n";
    j += "  \"version\": " + std::to_string(kFormatVersion) + ",\n";
    j += "  \"topics\": {\n";
    const auto topic = [](const char* name, std::size_t n) {
      return std::string("    \"") + name + "\": {\"file\": \"" + name +
             ".bin\", \"count\": " + std::to_string(n) + "},\n";
    };
    j += topic("imu", imu.count);
    j += topic("gnss", gnss.count);
    j += topic("odom", odom.count);
    j += topic("gt", gt.count);
    j += "    \"lidar\": {\"dir\": \"lidar\", \"count\": " + std::to_string(n_lidar) + "}\n";
    j += "  },\n";
    j += "  \"origin_lla\": " +
         json_array({meta.origin_lla.lat_deg, meta.origin_lla.lon_deg, meta.origin_lla.alt_m}) +
         ",\n";
    // Quaternion in Hamilton order (w, x, y, z), as docs/mssdata_format.md requires.
    j += "  \"T_b_l\": {\"translation_m\": " +
         json_array({meta.T_b_l.p.x(), meta.T_b_l.p.y(), meta.T_b_l.p.z()}) +
         ", \"quaternion_wxyz\": " + json_array({q.w(), q.x(), q.y(), q.z()}) + "},\n";
    j += "  \"p_b_g_m\": " + json_array({meta.p_b_g.x(), meta.p_b_g.y(), meta.p_b_g.z()}) + ",\n";
    j += "  \"body_frame\": " + json_string(meta.body_frame) + ",\n";
    j += "  \"lidar_frame\": " + json_string(meta.lidar_frame) + ",\n";
    j += std::string("  \"gnss_cov_filled_from_sigma\": ") +
         (meta.gnss_cov_filled_from_sigma ? "true" : "false") + "\n";
    j += "}\n";

    const std::filesystem::path path = dir / "index.json";
    FilePtr f(std::fopen(path.c_str(), "wb"));
    if (!f) {
      return Status::error(path.string() + ": cannot create");
    }
    if (std::fwrite(j.data(), 1, j.size(), f.get()) != j.size()) {
      return Status::error(path.string() + ": write failed");
    }
    if (std::fclose(f.release()) != 0) {
      return Status::error(path.string() + ": close failed");
    }
    return Status::ok();
  }
};

DatasetWriter::DatasetWriter(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
DatasetWriter::DatasetWriter(DatasetWriter&& other) noexcept = default;

DatasetWriter& DatasetWriter::operator=(DatasetWriter&& other) noexcept {
  if (this != &other) {
    if (impl_) {
      finish();
    }
    impl_ = std::move(other.impl_);
  }
  return *this;
}

DatasetWriter::~DatasetWriter() {
  if (impl_) {
    finish();
  }
}

Result<DatasetWriter> DatasetWriter::create(const std::filesystem::path& dir,
                                            const DatasetIndex& meta) {
  using R = Result<DatasetWriter>;
  if (!host_is_little_endian()) {
    return R::error("DatasetWriter: mssdata is little-endian; big-endian hosts are unsupported");
  }
  const common::Mat3& Rbl = meta.T_b_l.R;
  if (!all_finite({meta.origin_lla.lat_deg,
                   meta.origin_lla.lon_deg,
                   meta.origin_lla.alt_m,
                   meta.T_b_l.p.x(),
                   meta.T_b_l.p.y(),
                   meta.T_b_l.p.z(),
                   meta.p_b_g.x(),
                   meta.p_b_g.y(),
                   meta.p_b_g.z()}) ||
      !Rbl.allFinite()) {
    return R::error("DatasetWriter: non-finite metadata (origin_lla, T_b_l or p_b_g)");
  }

  std::error_code ec;
  if (std::filesystem::exists(dir, ec)) {
    if (!std::filesystem::is_directory(dir, ec) || !std::filesystem::is_empty(dir, ec) || ec) {
      return R::error(dir.string() + ": exists and is not an empty directory");
    }
  }
  std::filesystem::create_directories(dir / "lidar", ec);
  if (ec) {
    return R::error((dir / "lidar").string() + ": cannot create directory: " + ec.message());
  }

  auto impl = std::make_unique<Impl>();
  impl->dir = dir;
  impl->meta = meta;
  const auto open_stream = [&dir](Impl::Stream& s, const char* name) {
    const std::filesystem::path p = dir / name;
    s.path = p.string();
    s.file.reset(std::fopen(p.c_str(), "wb"));
    return static_cast<bool>(s.file);
  };
  if (!open_stream(impl->imu, "imu.bin") || !open_stream(impl->gnss, "gnss.bin") ||
      !open_stream(impl->odom, "odom.bin") || !open_stream(impl->gt, "gt.bin")) {
    return R::error(dir.string() + ": cannot create the record files");
  }
  return R::ok(DatasetWriter(std::move(impl)));
}

Status DatasetWriter::write_imu(const common::ImuSample& s) {
  if (Status st = impl_->check_open("write_imu"); !st.is_ok()) {
    return st;
  }
  if (Status st = Impl::check_time(impl_->imu, s.t.ns); !st.is_ok()) {
    return st;
  }
  ImuRecord rec{};
  rec.t = s.t.ns;
  for (int i = 0; i < 3; ++i) {
    rec.acc[i] = s.acc_b(i);
    rec.gyro[i] = s.gyro_b(i);
  }
  return Impl::append(impl_->imu, rec);
}

Status DatasetWriter::write_gnss(const common::GnssMeasurement& m) {
  if (Status st = impl_->check_open("write_gnss"); !st.is_ok()) {
    return st;
  }
  if (Status st = Impl::check_time(impl_->gnss, m.t.ns); !st.is_ok()) {
    return st;
  }
  GnssRecord rec{};
  rec.t = m.t.ns;
  for (int i = 0; i < 3; ++i) {
    rec.p_e_g[i] = m.p_e_g(i);
    for (int j = 0; j < 3; ++j) {
      rec.cov[3 * i + j] = m.cov(i, j);  // row-major on disk
    }
  }
  rec.status = m.status;
  return Impl::append(impl_->gnss, rec);
}

Status DatasetWriter::write_odom(const common::WheelSample& s) {
  if (Status st = impl_->check_open("write_odom"); !st.is_ok()) {
    return st;
  }
  if (Status st = Impl::check_time(impl_->odom, s.t.ns); !st.is_ok()) {
    return st;
  }
  OdomRecord rec{};
  rec.t = s.t.ns;
  rec.v = s.v_mps;
  rec.omega = s.omega_radps;
  return Impl::append(impl_->odom, rec);
}

Status DatasetWriter::write_gt(const common::NavState& s) {
  if (Status st = impl_->check_open("write_gt"); !st.is_ok()) {
    return st;
  }
  if (Status st = Impl::check_time(impl_->gt, s.t.ns); !st.is_ok()) {
    return st;
  }
  GtRecord rec{};
  rec.t = s.t.ns;
  for (int i = 0; i < 3; ++i) {
    rec.p[i] = s.T_w_b.p(i);
  }
  // R_w_b as a Hamilton quaternion stored in order (w, x, y, z).
  const Eigen::Quaterniond q = quaternion_of(s.T_w_b.R);
  rec.q_wxyz[0] = q.w();
  rec.q_wxyz[1] = q.x();
  rec.q_wxyz[2] = q.y();
  rec.q_wxyz[3] = q.z();
  return Impl::append(impl_->gt, rec);
}

Status DatasetWriter::write_lidar(const common::ScanFrame& scan) {
  if (Status st = impl_->check_open("write_lidar"); !st.is_ok()) {
    return st;
  }
  Impl& d = *impl_;
  const std::size_t n = scan.size();
  if (n > std::numeric_limits<std::uint32_t>::max()) {
    return Status::error("write_lidar: scan has too many points for a uint32 count");
  }
  char name[32];
  std::snprintf(name, sizeof(name), "%0*zu.bin", static_cast<int>(kLidarNameWidth), d.n_lidar);
  const std::filesystem::path path = d.dir / "lidar" / name;
  if (scan.t_start.ns < d.last_lidar_t) {
    return Status::error(path.string() + ": t_start " + std::to_string(scan.t_start.ns) +
                         " ns is before the previous " + std::to_string(d.last_lidar_t) + " ns");
  }

  LidarHeader header{};
  header.t_start = scan.t_start.ns;
  header.n = static_cast<std::uint32_t>(n);
  if (d.staging.size() < n) {
    d.staging.resize(n);
  }
  const float* xyz = scan.xyz().data();
  const float* intensity = scan.intensity().data();
  const std::uint32_t* t_off = scan.t_offset_ns().data();
  const std::uint16_t* ring = scan.ring().data();
  for (std::size_t i = 0; i < n; ++i) {
    LidarPoint& p = d.staging[i];
    p.xyz[0] = xyz[3 * i];
    p.xyz[1] = xyz[3 * i + 1];
    p.xyz[2] = xyz[3 * i + 2];
    p.intensity = intensity[i];
    p.t_offset_ns = t_off[i];
    p.ring = ring[i];
  }

  FilePtr f(std::fopen(path.c_str(), "wb"));
  if (!f) {
    return Status::error(path.string() + ": cannot create");
  }
  if (std::fwrite(&header, sizeof(header), 1, f.get()) != 1 ||
      (n > 0 && std::fwrite(d.staging.data(), sizeof(LidarPoint), n, f.get()) != n)) {
    return Status::error(path.string() + ": write failed");
  }
  if (std::fclose(f.release()) != 0) {
    return Status::error(path.string() + ": close failed");
  }
  d.last_lidar_t = scan.t_start.ns;
  ++d.n_lidar;
  return Status::ok();
}

Status DatasetWriter::finish() {
  if (!impl_) {
    return Status::error("DatasetWriter: finish() on a moved-from writer");
  }
  Impl& d = *impl_;
  if (d.finished) {
    return d.finish_status;
  }
  d.finished = true;
  std::string failed;
  for (Impl::Stream* s : {&d.imu, &d.gnss, &d.odom, &d.gt}) {
    if (s->file && std::fclose(s->file.release()) != 0) {
      failed += (failed.empty() ? "" : ", ") + s->path;
    }
  }
  if (!failed.empty()) {
    d.finish_status = Status::error("DatasetWriter: close failed: " + failed);
    return d.finish_status;
  }
  d.finish_status = d.write_index();
  return d.finish_status;
}

}  // namespace mss::io
