#include <cmath>

#include "mss/gnss/gnss.hpp"

namespace mss::gnss {

namespace {

// WGS-84 algorithm constants (not tunables), copied from sim/mss_sim/geodesy.py so the
// C++ and Python conversions agree (G-01). See docs/derivations/gnss.md section 1.
constexpr double kWgs84AM = 6378137.0;                   // m, semi-major axis
constexpr double kWgs84F = 1.0 / 298.257223563;          // flattening
constexpr double kWgs84BM = kWgs84AM * (1.0 - kWgs84F);  // m, semi-minor axis
constexpr double kE2 = kWgs84F * (2.0 - kWgs84F);        // first eccentricity squared
constexpr double kEp2 = kE2 / (1.0 - kE2);               // second eccentricity squared
constexpr int kBowringIterations = 5;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

common::Vec3 lla_to_ecef(const Lla& lla) {
  const double lat = lla.lat_deg * kDegToRad;
  const double lon = lla.lon_deg * kDegToRad;
  const double h = lla.alt_m;
  const double sin_lat = std::sin(lat);
  const double n = kWgs84AM / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
  return {(n + h) * std::cos(lat) * std::cos(lon),
          (n + h) * std::cos(lat) * std::sin(lon),
          (n * (1.0 - kE2) + h) * sin_lat};
}

Lla ecef_to_lla(const common::Vec3& xyz) {
  const double x = xyz.x();
  const double y = xyz.y();
  const double z = xyz.z();
  const double p = std::hypot(x, y);
  const double lon = std::atan2(y, x);
  double beta = std::atan2(z * kWgs84AM, p * kWgs84BM);
  double lat = std::atan2(z, p * (1.0 - kE2));
  for (int i = 0; i < kBowringIterations; ++i) {
    const double sb = std::sin(beta);
    const double cb = std::cos(beta);
    lat = std::atan2(z + kEp2 * kWgs84BM * sb * sb * sb, p - kE2 * kWgs84AM * cb * cb * cb);
    beta = std::atan2((1.0 - kWgs84F) * std::sin(lat), std::cos(lat));
  }
  const double sin_lat = std::sin(lat);
  const double n = kWgs84AM / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
  Lla out;
  out.lat_deg = lat * kRadToDeg;
  out.lon_deg = lon * kRadToDeg;
  out.alt_m = p * std::cos(lat) + z * sin_lat - kWgs84AM * kWgs84AM / n;
  return out;
}

common::Mat3 r_e_ecef(const Lla& origin) {
  const double lat = origin.lat_deg * kDegToRad;
  const double lon = origin.lon_deg * kDegToRad;
  const double sl = std::sin(lat);
  const double cl = std::cos(lat);
  const double so = std::sin(lon);
  const double co = std::cos(lon);
  common::Mat3 R_e_ecef;
  R_e_ecef << -so, co, 0.0,    //
      -sl * co, -sl * so, cl,  //
      cl * co, cl * so, sl;
  return R_e_ecef;
}

}  // namespace

common::Vec3 lla_to_enu(const Lla& lla, const Lla& origin) {
  const common::Vec3 d = lla_to_ecef(lla) - lla_to_ecef(origin);
  const common::Mat3 R_e_ecef = r_e_ecef(origin);
  return common::Vec3(R_e_ecef * d);
}

Lla enu_to_lla(const common::Vec3& p_e, const Lla& origin) {
  const common::Mat3 R_e_ecef = r_e_ecef(origin);
  const common::Vec3 ecef = lla_to_ecef(origin) + R_e_ecef.transpose() * p_e;
  return ecef_to_lla(ecef);
}

}  // namespace mss::gnss
