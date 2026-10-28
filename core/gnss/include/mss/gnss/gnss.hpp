#pragma once

#include <array>
#include <limits>

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
/// along its forward axis x_b, forwards or backwards. The heading psi of R_w_b x_b (horizontal,
/// w) is unwrapped from fix to fix. Fixes are grouped in chunks of
/// GnssConfig::align_yaw_chunk_fixes; per chunk the mean horizontal fix position c_k (e), the
/// mean heading psi_k, its span, the mean horizontal lever arm m_k = mean (R_w_b p_b_g) (w) and
/// the signed wheel travel of each half. Segment k joins chunks k and k+1:
///   b_k = c_{k+1} - c_k,  l_k = m_{k+1} - m_k,  a_k = s_k (cos, sin)((psi_k + psi_{k+1}) / 2)
/// Turn gate: a segment whose heading change, or the span of either chunk, exceeds
/// align_yaw_max_turn_rad is skipped (a_k = 0, n_skipped_turns()).
/// Direction s_k = +-1: the sign of the wheel travel between the two centroids (second half
/// of chunk k + first half of chunk k+1) when every fix there came with wheel data (a segment
/// with less than align_yaw_min_wheel_travel_m of wheel travel is stationary and skipped,
/// n_stationary_segments()); otherwise
/// the forward-driving assumption (n_no_wheel_segments()), corrected once confident when the
/// segment clearly goes backwards (more than align_yaw_reverse_nsigma of its sigma).
/// With the lever-arm rotation removed, b'_k = b_k - Rz(yaw) l_k, the yaw solves
///   yaw = arg(sum conj(a_k) b'_k)      (complex notation, 2-D Procrustes)
/// in closed form: with Z = sum conj(a) b and L = sum conj(a) l,
///   r = -Re(L) + sqrt(|Z|^2 - Im(L)^2),  e^{i yaw} = Z / (r + L),  r = |sum conj(a) b'| > 0.
/// Noise: sigma_yaw = sqrt(sum_k var_k |a_{k-1} - a_k|^2) / |Z| (|Z| = |r + L|), var_k the per-axis
/// variance of chunk mean k from the declared fix variances floored at align_yaw_min_var_m2
/// (fixes assumed white and independent), a_{-1} = a_K = 0.
/// A transform is returned once confident: r >= align_yaw_min_track_m,
/// r >= align_yaw_min_coherence * sum |b_k| (segments used) and sigma_yaw <=
/// align_yaw_max_sigma_rad.
/// Not handled: without wheel data, net backward motion along the heading before the estimate
/// is confident gives yaw + 180 deg; a parked robot whose GNSS wanders by 0.1 m per fix or
/// more can align on noise (gnss.md section 5). With wheel data, a robot travelling less than
/// align_yaw_min_wheel_travel_m per chunk interval (about 0.1 m/s at the defaults) never aligns.
/// Coloured GNSS error is not in sigma_yaw; the minimum track and the coherence test bound its
/// effect. Translation: p_e_w = mean(p_e_g) - R_e_w mean(p_w_g) over all fixes, as GnssAligner.
///
/// O(1) memory, no allocation. Ownership/threading: single-threaded; owned by
/// pipeline::Estimator (backend thread) when gnss.align_yaw_from_track is true.
class GnssTrackYawAligner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Stores the align_yaw_* thresholds (validated by load_config). Errors: none. Hot path: no.
  /// Tests: test_gnss_track_yaw.
  explicit GnssTrackYawAligner(const GnssConfig& cfg);

  /// Adds one fix and returns the current `T_e_w` estimate when available.
  /// Frames: `p_e_g` antenna in e, `cov_e` its covariance in e (m^2; only the mean of its two
  ///   horizontal diagonal terms is used, floored at align_yaw_min_var_m2); `T_w_b` body pose
  ///   in w at the fix time; `p_b_g` lever arm in b. Result: T_e_w maps w to e.
  /// `wheel_odometer_m`: absolute signed wheel odometer reading at the fix (m, increasing when
  ///   driving forward), or NaN when no wheel data covers the interval since the previous fix.
  ///   The travel of a fix is the difference to the reading of the last fix added here, so the
  ///   travel during fixes rejected before the aligner is carried to the next one.
  /// Errors (Result::error), first failure returned:
  ///   1. "non-finite input" (wheel_odometer_m may be NaN, not infinite; fix NOT added);
  ///   2. "non-positive covariance" when cov_e(0,0) <= 0 or cov_e(1,1) <= 0 (fix NOT added);
  ///   3. "heading undefined" when the body x axis is within about 1e-6 rad of vertical
  ///      (fix NOT added);
  ///   4. "not enough track" until two chunks are complete and the closed form has a solution
  ///      with r > 0 (fix IS added);
  ///   5. "track too short" while r < align_yaw_min_track_m (fix IS added);
  ///   6. "track not coherent" while r < align_yaw_min_coherence * sum |b_k| (fix IS added);
  ///   7. "yaw sigma too large" while sigma_yaw > align_yaw_max_sigma_rad (fix IS added).
  /// The yaw is re-estimated when a chunk completes; between chunks the last yaw is returned with
  /// the translation updated by the new fix.
  /// Hot path: no (per fix); no allocation, no logging. Tests: test_gnss_track_yaw.
  common::Result<common::Pose> update(
      const common::Vec3& p_e_g,
      const common::Mat3& cov_e,
      const common::Pose& T_w_b,
      const common::Vec3& p_b_g,
      double wheel_odometer_m = std::numeric_limits<double>::quiet_NaN());

  /// Predicted 1-sigma of the yaw (rad) at the last chunk boundary; +inf before it is defined.
  [[nodiscard]] double yaw_sigma_rad() const { return yaw_sigma_rad_; }
  /// Along-heading GNSS track length r (m) supporting the yaw at the last chunk boundary.
  [[nodiscard]] double track_m() const { return track_m_; }
  /// Segments taken as reverse motion so far (wheel sign, or the no-wheel reversal test).
  [[nodiscard]] int n_reversals() const { return n_reversals_; }
  /// Segments skipped by the turn gate.
  [[nodiscard]] int n_skipped_turns() const { return n_skipped_turns_; }
  /// Segments skipped as stationary (wheel travel below align_yaw_min_wheel_travel_m).
  [[nodiscard]] int n_stationary_segments() const { return n_stationary_; }
  /// Segments whose direction came from the forward-driving assumption (no wheel data).
  [[nodiscard]] int n_no_wheel_segments() const { return n_no_wheel_segments_; }

 private:
  struct Chunk {
    Eigen::Vector2d c = Eigen::Vector2d::Zero();  // m, mean position (shifted by ref_e_)
    Eigen::Vector2d m = Eigen::Vector2d::Zero();  // m, mean horizontal lever arm, w
    double psi = 0.0;                             // rad, mean unwrapped heading
    double span = 0.0;                            // rad, max - min unwrapped heading
    double var = 0.0;                             // m^2, per-axis variance of c
    double wheel_first_m = 0.0;                   // m, signed wheel travel, first half
    double wheel_second_m = 0.0;                  // m, signed wheel travel, second half
    bool wheel_first_ok = false;                  // every fix of the first half had wheel data
    bool wheel_second_ok = false;
  };
  void close_chunk();
  [[nodiscard]] bool confident() const {
    return have_yaw_ && track_m_ >= min_track_m_ && track_m_ >= min_coherence_ * sum_abs_b_m_ &&
           yaw_sigma_rad_ <= max_sigma_rad_;
  }

  int chunk_fixes_;            // fixes per chunk, from GnssConfig
  double max_sigma_rad_;       // rad, from GnssConfig
  double min_track_m_;         // m, from GnssConfig
  double min_coherence_;       // ratio, from GnssConfig
  double reverse_nsigma_;      // sigmas, from GnssConfig
  double max_turn_rad_;        // rad, from GnssConfig
  double min_var_m2_;          // m^2, from GnssConfig
  double min_wheel_travel_m_;  // m, from GnssConfig
  // Unwrapped heading of the last fix.
  bool have_psi_ = false;
  double psi_rad_ = 0.0;                                // rad, unwrapped
  Eigen::Vector2d last_fwd_ = Eigen::Vector2d::Zero();  // unit heading of the last fix, w
  // Current chunk (positions shifted by ref_e_ for conditioning).
  int n_chunk_ = 0;
  Eigen::Vector2d chunk_sum_e_ = Eigen::Vector2d::Zero();  // m
  double chunk_sum_psi_rad_ = 0.0;                         // rad, unwrapped headings
  double chunk_min_psi_rad_ = 0.0;                         // rad
  double chunk_max_psi_rad_ = 0.0;                         // rad
  Eigen::Vector2d chunk_sum_l_ = Eigen::Vector2d::Zero();  // m, horizontal R_w_b p_b_g, w
  double chunk_sum_var_m2_ = 0.0;                          // m^2, floored per-fix variances
  std::array<double, 2> chunk_wheel_m_{0.0, 0.0};          // m, signed wheel travel per half
  double last_odometer_m_ = std::numeric_limits<double>::quiet_NaN();  // m, at the last fix added
  std::array<bool, 2> chunk_wheel_ok_{true, true};  // every fix of the half had wheel
  // Previous complete chunk and the last segment direction.
  bool have_prev_ = false;
  Chunk prev_;
  Eigen::Vector2d prev_a_ = Eigen::Vector2d::Zero();  // last segment direction (0 before one)
  // Segment sums: Z = sum conj(a) b, L = sum conj(a) l (real part: dot, imaginary: cross).
  double sum_dot_m_ = 0.0;       // m, sum a . b
  double sum_cross_m_ = 0.0;     // m, sum a x b
  double sum_al_dot_m_ = 0.0;    // m, sum a . l
  double sum_al_cross_m_ = 0.0;  // m, sum a x l
  double sum_abs_b_m_ = 0.0;     // m, sum |b| over the segments used
  double noise_q_m2_ = 0.0;      // m^2, sum var_k |a_{k-1} - a_k|^2 over closed coefficients
  int n_reversals_ = 0;
  int n_skipped_turns_ = 0;
  int n_no_wheel_segments_ = 0;
  int n_stationary_ = 0;
  double yaw_rad_ = 0.0;
  double yaw_sigma_rad_ = 0.0;  // set to +inf in the constructor
  double track_m_ = 0.0;        // m, r of the last solution
  bool have_yaw_ = false;
  // Translation: all fixes, shifted by the first one (as GnssAligner).
  int n_fixes_ = 0;
  common::Vec3 ref_w_ = common::Vec3::Zero();  // m, first p_w_g
  common::Vec3 ref_e_ = common::Vec3::Zero();  // m, first p_e_g
  common::Vec3 sum_w_ = common::Vec3::Zero();  // m
  common::Vec3 sum_e_ = common::Vec3::Zero();  // m
};

}  // namespace mss::gnss
