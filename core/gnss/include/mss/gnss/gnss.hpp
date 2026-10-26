#pragma once

#include <Eigen/Core>

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
/// Core treats `m.cov` as valid as delivered. Filling it from
/// GnssConfig::sigma_h_m / sigma_v_m when a NavSatFix carries no covariance
/// (`position_covariance_type == 0`) is the adapter's job: `ros2_node` for live/bag
/// playback and `tools/bag_to_mssdata.py` for datasets.
/// Frames: `m.p_e_g` in e. Units: `max_cov_m2` in m^2 (a variance, per axis).
/// Errors: returns Status::error; checks run in the listed order and the first failure
///   is returned:
///   1. `m.status < 0` ("no fix");
///   2. any element of `m.p_e_g` or `m.cov` is non-finite ("non-finite fix");
///   3. any diagonal element of `m.cov` is > `max_cov_m2` ("fix covariance too large").
///   Otherwise Status::ok().
/// Hot path: no (per fix); no allocation on the success path and on error paths that use
/// string literals (Status::error(const char*)), no logging (the caller logs the reason).
/// Tests: G-02 (rejects bad fixes before the chi-square stage), gnss unit tests.
common::Status gate_fix(const common::GnssMeasurement& m, double max_cov_m2);

/// Chi-square quantile, 3 DoF, q = 0.95 (docs/derivations/gnss.md section 3): the default of
/// GnssConfig::chi2_gate (YAML gnss.chi2_gate).
constexpr double kChi2Gate3Dof95 = 7.814727903251178;

/// GNSS position residual and its Jacobian (docs/derivations/gnss.md section 2).
///   Residual: r = p_e_g - T_e_w * (T_w_b * p_b_g)   (3x1, m, expressed in e).
///   Variable: T_w_b, right perturbation T_w_b <- T_w_b * Exp(xi), xi = [rho, phi].
///   J_r_xi = [ -R_e_w R_w_b ,  R_e_w R_w_b skew(p_b_g) ]   (3x6).
/// Frames: p_e_g antenna in e; T_e_w maps w to e; T_w_b body in w; p_b_g lever arm in b.
/// `J_r_xi` may be nullptr. Errors: none. Hot path: no (per fix); no allocation.
/// Tests: G-02 (test_gnss_gate, Jacobian vs numerical differentiation).
common::Vec3 gnss_residual(const common::Vec3& p_e_g,
                           const common::Pose& T_e_w,
                           const common::Pose& T_w_b,
                           const common::Vec3& p_b_g,
                           Eigen::Matrix<double, 3, 6>* J_r_xi);

/// Normalised innovation squared r^T S^-1 r (S in m^2, r in m, both in e).
/// Errors: "non-finite innovation" (r or S non-finite), "innovation covariance not
/// positive definite" (LLT fails). No allocation. Tests: G-02.
common::Result<double> innovation_nis(const common::Vec3& r_e, const common::Mat3& s_e);

/// Chi-square gate (G-02): ok when NIS <= threshold (the pipeline passes
/// GnssConfig::chi2_gate). Errors: those of innovation_nis, then "chi-square gate
/// exceeded". No allocation, no logging. Tests: G-02.
common::Status chi_square_gate(const common::Vec3& r_e, const common::Mat3& s_e, double threshold);

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Default of GnssConfig::align_min_pairs: minimum number of accepted pairs before
  /// update() returns a transform. The aligner uses the configured value.
  static constexpr int kMinPairsForAlignment = 10;
  /// Default of GnssConfig::align_min_spread_m: minimum RMS horizontal distance (m) of the
  /// antenna positions from their centroid, required on both the w side and the e side;
  /// below it yaw is unobservable. The aligner uses the configured value.
  static constexpr double kMinHorizontalSpreadM = 1.0;

  /// Stores `cfg.align_min_pairs` and `cfg.align_min_spread_m` (validated by load_config).
  /// Errors: none. Hot path: no.
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
  ///   2. "not enough pairs" while fewer than GnssConfig::align_min_pairs pairs were
  ///      added, counting this one (the pair IS added);
  ///   3. "insufficient horizontal spread" while the w-side or the e-side spread is
  ///      < GnssConfig::align_min_spread_m (the pair IS added).
  /// Hot path: no (per fix); no allocation on the success path and on error paths that
  /// use string literals, no logging.
  /// Tests: G-03 (yaw error < 0.5 deg after 30 m), G-04 (lever arm).
  common::Result<common::Pose> update(const common::Vec3& p_e_g,
                                      const common::Pose& T_w_b,
                                      const common::Vec3& p_b_g);

 private:
  // Running sums over all accepted pairs, shifted by the first pair for conditioning
  // (docs/derivations/gnss.md section 4). a = p_w_g - ref_w_, b = p_e_g - ref_e_.
  int min_pairs_ = kMinPairsForAlignment;        // pairs, from GnssConfig
  double min_spread_m_ = kMinHorizontalSpreadM;  // m, from GnssConfig
  int n_pairs_ = 0;
  common::Vec3 ref_w_ = common::Vec3::Zero();         // m, first p_w_g
  common::Vec3 ref_e_ = common::Vec3::Zero();         // m, first p_e_g
  common::Vec3 sum_w_ = common::Vec3::Zero();         // m, sum a
  common::Vec3 sum_e_ = common::Vec3::Zero();         // m, sum b
  Eigen::Matrix2d sum_we_ = Eigen::Matrix2d::Zero();  // m^2, sum a_h b_h^T
  double sum_w_h_sq_m2_ = 0.0;                        // m^2, sum |a_h|^2
  double sum_e_h_sq_m2_ = 0.0;                        // m^2, sum |b_h|^2
};

}  // namespace mss::gnss
