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

/// ADR-023: estimates `T_e_w` with its yaw taken from the direction of the GNSS track (e side)
/// against the body heading in w, without the front end's along-track positions
/// (docs/derivations/gnss.md section 5).
///
/// Why: along a LiDAR-degenerate axis the front-end positions are IMU dead reckoning, whose
/// along-axis velocity can even change sign; GnssAligner fits them and can start 180 deg off
/// (gnss_fusion_study.md section 13). The heading (gyro + LiDAR rotation) stays accurate there.
///
/// Model: R_e_w = Rz(yaw) (4 DoF, w and e gravity-aligned, as GnssAligner); the body moves
/// along its forward axis x_b. Fixes are grouped in chunks of GnssConfig::align_yaw_chunk_fixes
/// consecutive fixes; per chunk the mean horizontal fix position c_k (e) and the mean unit
/// horizontal heading of R_w_b x_b (w). Segment k joins chunks k and k+1: b_k = c_{k+1} - c_k,
/// a_k = unit(h_k + h_{k+1}) (times -1 when a reversal is detected, below), and
///   yaw = atan2(sum a_k x b_k, sum a_k . b_k)      (2-D cross / dot products).
/// The chunk means telescope along a straight line, so the noise of the estimate is that of the
/// first and last chunk means plus the heading changes in between:
///   sigma_yaw = sqrt(sum_k var_k |a_{k-1} - a_k|^2) / |(sum a.b, sum a x b)|
/// with var_k the per-axis variance of chunk mean k (mean of the fixes' horizontal variances / n)
/// and a_{-1} = a_K = 0. A transform is returned once sigma_yaw <= align_yaw_max_sigma_rad.
/// Sign: the body is assumed to move forward at the start. Once the estimate is confident
/// (sigma_yaw <= the threshold), a new segment whose along-heading component, rotated into e,
/// is below -align_yaw_reverse_nsigma times its own sigma is a reversal: its a_k is negated
/// (counted in n_reversals()). Turns are handled by the per-segment heading.
/// Translation: p_e_w = mean(p_e_g) - R_e_w mean(p_w_g) over all fixes, as GnssAligner.
/// Lever arm: p_w_g = T_w_b p_b_g in the translation. The antenna's rotation about the body
/// origin, l_k = mean(R_w_b p_b_g)_{k+1} - mean(R_w_b p_b_g)_k (horizontal, w), is removed from
/// each segment as b_k - Rz(yaw) l_k, solved by a fixed-point iteration on two more running sums.
/// Not handled: a robot that drives backwards from the start aligns 180 deg off.
///
/// O(1) memory, no allocation. Ownership/threading: single-threaded; owned by
/// pipeline::Estimator (backend thread) when gnss.align_yaw_from_track is true.
class GnssTrackYawAligner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Stores align_yaw_chunk_fixes, align_yaw_max_sigma_rad and align_yaw_reverse_nsigma
  /// (validated by load_config). Errors: none. Hot path: no. Tests: test_gnss_track_yaw.
  explicit GnssTrackYawAligner(const GnssConfig& cfg);

  /// Adds one fix and returns the current `T_e_w` estimate when available.
  /// Frames: `p_e_g` antenna in e, `cov_e` its covariance in e (m^2; only the mean of its two
  ///   horizontal diagonal terms is used); `T_w_b` body pose in w at the fix time; `p_b_g`
  ///   lever arm in b. Result: T_e_w maps w to e.
  /// Errors (Result::error), first failure returned:
  ///   1. "non-finite input" (fix NOT added);
  ///   2. "heading undefined" when the body x axis is within about 1e-6 rad of vertical
  ///      (fix NOT added);
  ///   3. "not enough track" until two chunks are complete and their segment has a non-zero
  ///      along-track sum (fix IS added);
  ///   4. "yaw sigma too large" while sigma_yaw > align_yaw_max_sigma_rad (fix IS added).
  /// The yaw is re-estimated when a chunk completes; between chunks the last yaw is returned with
  /// the translation updated by the new fix.
  /// Hot path: no (per fix); no allocation, no logging. Tests: test_gnss_track_yaw.
  common::Result<common::Pose> update(const common::Vec3& p_e_g,
                                      const common::Mat3& cov_e,
                                      const common::Pose& T_w_b,
                                      const common::Vec3& p_b_g);

  /// Predicted 1-sigma of the yaw (rad) at the last chunk boundary; +inf before it is defined.
  [[nodiscard]] double yaw_sigma_rad() const { return yaw_sigma_rad_; }
  /// Segments taken as reverse motion so far.
  [[nodiscard]] int n_reversals() const { return n_reversals_; }

 private:
  void close_chunk();

  int chunk_fixes_;        // fixes per chunk, from GnssConfig
  double max_sigma_rad_;   // rad, from GnssConfig
  double reverse_nsigma_;  // sigmas, from GnssConfig
  // Current chunk (positions shifted by ref_e_ for conditioning).
  int n_chunk_ = 0;
  Eigen::Vector2d chunk_sum_e_ = Eigen::Vector2d::Zero();  // m
  Eigen::Vector2d chunk_sum_h_ = Eigen::Vector2d::Zero();  // unit headings, w
  Eigen::Vector2d chunk_sum_l_ = Eigen::Vector2d::Zero();  // m, horizontal R_w_b p_b_g, w
  double chunk_sum_var_m2_ = 0.0;                          // m^2, sum of per-fix variances
  // Previous complete chunk and the last segment direction.
  bool have_prev_ = false;
  Eigen::Vector2d prev_c_ = Eigen::Vector2d::Zero();  // m, mean position (shifted)
  Eigen::Vector2d prev_m_ = Eigen::Vector2d::Zero();  // m, mean horizontal lever arm, w
  Eigen::Vector2d prev_h_ = Eigen::Vector2d::Zero();  // unit mean heading, w
  double prev_var_m2_ = 0.0;                          // m^2, per-axis variance of prev_c_
  Eigen::Vector2d prev_a_ = Eigen::Vector2d::Zero();  // last segment direction (0 before one)
  // Segment sums (b uncorrected; the lever-arm correction uses the two sums of a and l).
  double sum_dot_m_ = 0.0;       // m, sum a . b
  double sum_cross_m_ = 0.0;     // m, sum a x b
  double sum_al_dot_m_ = 0.0;    // m, sum a . l (l: lever-arm displacement of the segment, w)
  double sum_al_cross_m_ = 0.0;  // m, sum a x l
  double noise_q_m2_ = 0.0;      // m^2, sum var_k |a_{k-1} - a_k|^2 over closed coefficients
  int n_reversals_ = 0;
  double yaw_rad_ = 0.0;
  double yaw_sigma_rad_ = 0.0;  // set to +inf in the constructor
  bool have_yaw_ = false;
  // Translation: all fixes, shifted by the first one (as GnssAligner).
  int n_fixes_ = 0;
  common::Vec3 ref_w_ = common::Vec3::Zero();  // m, first p_w_g
  common::Vec3 ref_e_ = common::Vec3::Zero();  // m, first p_e_g
  common::Vec3 sum_w_ = common::Vec3::Zero();  // m
  common::Vec3 sum_e_ = common::Vec3::Zero();  // m
};

}  // namespace mss::gnss
