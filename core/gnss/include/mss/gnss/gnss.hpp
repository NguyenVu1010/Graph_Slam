#pragma once

#include "mss/common/result.hpp"
#include "mss/common/types.hpp"
#include "mss/gnss/config.hpp"

namespace mss::gnss {

/// Geodetic position on the WGS-84 ellipsoid (I/O boundary type: degrees allowed).
struct Lla {
  double lat_deg = 0.0;  ///< deg, geodetic latitude, north positive
  double lon_deg = 0.0;  ///< deg, longitude, east positive
  double alt_m = 0.0;    ///< m, height above the WGS-84 ellipsoid (as NavSatFix)
};

/// Converts `lla` to the local ENU frame e whose origin is `origin` (geodetic -> ECEF ->
/// ENU at origin, WGS-84, exact, no flat-earth approximation).
/// Frames: result in e (x east, y north, z up), origin of e at `origin`.
/// Units: degrees in, metres out.
/// Preconditions: |lat_deg| <= 90 for both arguments; finite inputs.
/// Errors: none (pure function; non-finite in -> non-finite out).
/// Hot path: no (per GNSS fix, 5 Hz); no allocation.
/// Tests: G-01 (round trip with enu_to_lla < 1 mm).
common::Vec3 lla_to_enu(const Lla& lla, const Lla& origin);

/// Inverse of lla_to_enu: ENU point `p_e` (origin `origin`) to WGS-84 geodetic.
/// Frames: `p_e` in e. Units: metres in, degrees / metres out.
/// Preconditions/Errors/Hot path: as lla_to_enu.
/// Tests: G-01.
Lla enu_to_lla(const common::Vec3& p_e, const Lla& origin);

/// Cheap pre-gate of a GNSS fix before it reaches the aligner or the graph. This is NOT
/// the chi-square innovation test of G-02 (that needs the predicted state and lives in
/// the pipeline/backend).
/// The pipeline passes `GnssConfig::max_cov_m2`.
/// Frames: `m.p_e_g` in e. Units: `max_cov_m2` in m^2 (a variance, per axis).
/// Errors: returns Status::error; checks run in the listed order and the first failure
///   is returned:
///   1. `m.status < 0` ("no fix");
///   2. any element of `m.p_e_g` or `m.cov` is non-finite ("non-finite fix");
///   3. any diagonal element of `m.cov` is > `max_cov_m2` ("fix covariance too large").
///   Otherwise Status::ok().
/// Hot path: no (per fix); no allocation, no logging (the caller logs the reason).
/// Tests: G-02 (rejects bad fixes before the chi-square stage), gnss unit tests.
common::Status gate_fix(const common::GnssMeasurement& m, double max_cov_m2);

/// Estimates `T_e_w`, the transform from the SLAM world frame w to the ENU frame e, from
/// pairs of GNSS antenna positions (in e) and estimated antenna positions (in w).
///
/// Model: both w and e are gravity-aligned with z up, so T_e_w has 4 DoF:
/// `R_e_w = Rz(yaw)` and translation `p_e_w` (3-D). Least squares over ALL pairs received
/// so far, equal weights: yaw from the closed-form 2-D Procrustes solution on the
/// horizontal components of the centred pairs, `p_e_w = mean(p_e_g) - R_e_w *
/// mean(p_w_g)`. Implemented with running sums: O(1) memory, no allocation, every call
/// after alignment re-estimates T_e_w from all pairs.
///
/// Ownership/threading: single-threaded; owned by pipeline::Estimator (backend thread).
class GnssAligner {
 public:
  /// Minimum number of accepted pairs before update() returns a transform. Start value;
  /// the gnss module moves it to GnssConfig (YAML `gnss.align_min_pairs`) when it
  /// implements the aligner.
  static constexpr int kMinPairsForAlignment = 10;
  /// Minimum RMS horizontal distance (m) of the w-frame antenna positions from their
  /// centroid; below it yaw is unobservable. Start value; moved to GnssConfig (YAML
  /// `gnss.align_min_spread_m`) by the gnss module.
  static constexpr double kMinHorizontalSpreadM = 1.0;

  /// Errors: none (GnssConfig needs no validation here). Hot path: no.
  /// Tests: G-03, G-04 construct it.
  explicit GnssAligner(const GnssConfig& cfg);

  /// Adds one pair and returns the current `T_e_w` estimate when available.
  /// The pair is (`p_e_g`, `p_w_g = T_w_b * p_b_g`): the lever arm is applied here, so
  /// a robot rotating in place gives a fixed body position (G-04).
  /// Frames: `p_e_g` antenna in e; `T_w_b` body pose in w at the fix time (interpolated
  ///   by the caller); `p_b_g` antenna lever arm in b (frames.p_b_g). Result: T_e_w maps
  ///   w to e (`p_e = T_e_w * p_w`).
  /// Units: metres.
  /// Preconditions: the caller has passed the fix through gate_fix().
  /// Errors (Result::error). Checks run in the listed order; the first failure is
  ///   returned:
  ///   1. "non-finite input" (pair NOT added);
  ///   2. "not enough pairs" while fewer than kMinPairsForAlignment pairs were added,
  ///      counting this one (the pair IS added);
  ///   3. "insufficient horizontal spread" while the spread is < kMinHorizontalSpreadM
  ///      (the pair IS added).
  /// Hot path: no (per fix); no allocation, no logging.
  /// Tests: G-03 (yaw error < 0.5 deg after 30 m), G-04 (lever arm).
  common::Result<common::Pose> update(const common::Vec3& p_e_g,
                                      const common::Pose& T_w_b,
                                      const common::Vec3& p_b_g);
};

}  // namespace mss::gnss
