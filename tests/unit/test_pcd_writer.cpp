// Unit tests of io::write_pcd (core/io/include/mss/io/pcd_writer.hpp): binary PCD round trip
// (header and data parsed back) and the path validation errors.
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <system_error>
#include <unistd.h>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "mss/io/pcd_writer.hpp"

namespace mss::io {
namespace {

namespace fs = std::filesystem;

class PcdDir {
 public:
  PcdDir() {
    const std::string test = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    path_ = fs::temp_directory_path() /
            ("mss_pcd_" + test + "_" + std::to_string(static_cast<long>(::getpid())));
    std::error_code ec;
    fs::remove_all(path_, ec);
    fs::create_directories(path_);
  }
  ~PcdDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  PcdDir(const PcdDir&) = delete;
  PcdDir& operator=(const PcdDir&) = delete;
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

struct ParsedPcd {
  std::vector<std::string> header;  // lines up to and including "DATA binary"
  std::vector<Eigen::Vector3f> points;
  std::size_t trailing_bytes = 0;  // bytes after the declared points
};

// Minimal PCD reader for the writer's own layout: header lines, then POINTS x 12 bytes.
ParsedPcd parse(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  ParsedPcd out;
  std::size_t pos = 0;
  std::size_t n_points = 0;
  while (pos < bytes.size()) {
    const std::size_t eol = bytes.find('\n', pos);
    if (eol == std::string::npos) {
      break;
    }
    const std::string line = bytes.substr(pos, eol - pos);
    out.header.push_back(line);
    pos = eol + 1;
    if (line.rfind("POINTS ", 0) == 0) {
      n_points = std::stoul(line.substr(7));
    }
    if (line.rfind("DATA", 0) == 0) {
      break;
    }
  }
  for (std::size_t i = 0; i < n_points && pos + 12 <= bytes.size(); ++i, pos += 12) {
    float xyz[3];
    std::memcpy(xyz, bytes.data() + pos, sizeof(xyz));
    out.points.emplace_back(xyz[0], xyz[1], xyz[2]);
  }
  out.trailing_bytes = bytes.size() - pos;
  return out;
}

TEST(Pcd, RoundTripHeaderAndData) {
  const PcdDir dir;
  const std::vector<Eigen::Vector3f> pts = {
      {1.0F, -2.5F, 3.25F}, {0.0F, 0.0F, 0.0F}, {-1234.5F, 6789.125F, 1e-3F}};
  const fs::path file = dir.path() / "map.pcd";
  const common::Status st = write_pcd(file, pts);
  ASSERT_TRUE(st.is_ok()) << st.reason();
  const ParsedPcd p = parse(file);
  const std::vector<std::string> expected = {"# .PCD v0.7 - Point Cloud Data file format",
                                             "VERSION 0.7",
                                             "FIELDS x y z",
                                             "SIZE 4 4 4",
                                             "TYPE F F F",
                                             "COUNT 1 1 1",
                                             "WIDTH 3",
                                             "HEIGHT 1",
                                             "VIEWPOINT 0 0 0 1 0 0 0",
                                             "POINTS 3",
                                             "DATA binary"};
  EXPECT_EQ(p.header, expected);
  ASSERT_EQ(p.points.size(), pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_EQ(p.points[i], pts[i]) << i;  // bit-exact float round trip
  }
  EXPECT_EQ(p.trailing_bytes, 0U);
}

TEST(Pcd, EmptyCloudWritesValidHeader) {
  const PcdDir dir;
  const fs::path file = dir.path() / "empty.pcd";
  const std::vector<Eigen::Vector3f> none;
  ASSERT_TRUE(write_pcd(file, none).is_ok());
  const ParsedPcd p = parse(file);
  ASSERT_EQ(p.header.size(), 11U);
  EXPECT_EQ(p.header[6], "WIDTH 0");
  EXPECT_EQ(p.header[9], "POINTS 0");
  EXPECT_TRUE(p.points.empty());
  EXPECT_EQ(p.trailing_bytes, 0U);
}

TEST(Pcd, OverwritesExistingFile) {
  const PcdDir dir;
  const fs::path file = dir.path() / "map.pcd";
  const std::vector<Eigen::Vector3f> many(10, Eigen::Vector3f(1.0F, 2.0F, 3.0F));
  ASSERT_TRUE(write_pcd(file, many).is_ok());
  const std::vector<Eigen::Vector3f> one = {{4.0F, 5.0F, 6.0F}};
  ASSERT_TRUE(write_pcd(file, one).is_ok());
  const ParsedPcd p = parse(file);
  ASSERT_EQ(p.points.size(), 1U);
  EXPECT_EQ(p.points[0], one[0]);
  EXPECT_EQ(p.trailing_bytes, 0U);
}

TEST(Pcd, BadPathsReturnErrorNamingThePath) {
  const PcdDir dir;
  const std::vector<Eigen::Vector3f> pts = {{1.0F, 2.0F, 3.0F}};

  const common::Status empty = write_pcd(fs::path{}, pts);
  ASSERT_FALSE(empty.is_ok());
  EXPECT_NE(empty.reason().find("empty path"), std::string::npos) << empty.reason();

  const fs::path no_parent = dir.path() / "missing" / "map.pcd";
  const common::Status missing = write_pcd(no_parent, pts);
  ASSERT_FALSE(missing.is_ok());
  EXPECT_NE(missing.reason().find(no_parent.string()), std::string::npos) << missing.reason();
  EXPECT_NE(missing.reason().find("does not exist"), std::string::npos) << missing.reason();
  EXPECT_FALSE(fs::exists(no_parent));

  const common::Status is_dir = write_pcd(dir.path(), pts);
  ASSERT_FALSE(is_dir.is_ok());
  EXPECT_NE(is_dir.reason().find("is a directory"), std::string::npos) << is_dir.reason();
}

}  // namespace
}  // namespace mss::io
