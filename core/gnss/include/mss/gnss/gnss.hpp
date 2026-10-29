#pragma once

#include <algorithm>
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

  /// As update() above, with the declared fix covariance `cov_e` (3x3 in e, m^2) used for the
  /// predicted yaw 1-sigma (yaw_sigma_rad(), docs/derivations/gnss.md section 4.1): each fix
  /// counts with the larger eigenvalue of its horizontal 2x2 block, floored at
  /// GnssConfig::align_yaw_min_var_m2. The 3-argument update() uses that floor for every fix.
  /// With GnssConfig::align_check_yaw_sigma (backlog G1) one more error follows check 3:
  ///   4. "yaw sigma above threshold" while yaw_sigma_rad() > GnssConfig::align_yaw_max_sigma_rad
  ///      (the pair IS added).
  /// Errors: also "non-finite input" when `cov_e` has a non-finite entry (pair NOT added).
  /// Hot path: no (per fix); no allocation, no logging.
  /// Tests: GnssAligner.YawSigma*, GnssAligner.E01Like*.
  common::Result<common::Pose> update(const common::Vec3& p_e_g,
                                      const common::Mat3& cov_e,
                                      const common::Pose& T_w_b,
                                      const common::Vec3& p_b_g);

  /// Predicted yaw 1-sigma (rad) of the fit at the last update() that passed checks 2 and 3:
  /// sqrt(sum_i var_i |a_c,i|^2) / sum_i |a_c,i|^2, with a_c,i the centred horizontal w-side
  /// antenna positions and var_i the per-fix variance above. The w side (front end) is taken
  /// as exact, so along a LiDAR-degenerate axis it is optimistic (ADR-023). NaN before that.
  [[nodiscard]] double yaw_sigma_rad() const { return yaw_sigma_rad_; }

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
  // Yaw sigma (G1): per-fix horizontal variance var_i (m^2) weighted sums of the w side.
  bool check_yaw_sigma_ = false;                         // GnssConfig::align_check_yaw_sigma
  double max_yaw_sigma_rad_ = 0.0;                       // rad, GnssConfig::align_yaw_max_sigma_rad
  double min_var_m2_ = 0.0;                              // m^2, GnssConfig::align_yaw_min_var_m2
  double sum_var_m2_ = 0.0;                              // m^2, sum var_i
  Eigen::Vector2d sum_var_w_ = Eigen::Vector2d::Zero();  // m^3, sum var_i a_h
  double sum_var_w_sq_m4_ = 0.0;                         // m^4, sum var_i |a_h|^2
  double yaw_sigma_rad_ = std::numeric_limits<double>::quiet_NaN();  // rad
};

/// ADR-023: estimates `T_e_w` with its yaw taken from the direction of the GNSS track (e side)
/// against the body heading in w, without the front end's along-track positions
/// (docs/derivations/gnss.md section 5).
///
/// Why: along a LiDAR-degenerate axis the front-end positions are IMU dead reckoning, whose
/// along-axis velocity can even change sign; GnssAligner fits them and can start 180 deg off
/// (gnss_fusion_study.md section 13). The heading (gyro + LiDAR rotation) stays accurate there.
///
/// Model: R_e_w = Rz(yaw) (4 DoF, w and e gravity-aligned, as GnssAligner); the robot moves
/// along the x axis of its odometry frame o (T_b_o; o = b by default), forwards or backwards.
/// The heading psi of R_w_b R_b_o x (horizontal part, w; exact for any mount, the horizontal
/// projection of x_o) is unwrapped from fix to fix. Fixes are grouped in chunks of
/// GnssConfig::align_yaw_chunk_fixes; per chunk the mean horizontal fix position c_k (e), the
/// mean heading psi_k, its heading variation, the mean horizontal lever arm from o,
/// m_k = mean (R_w_b (p_b_g - p_b_o)) (w), and the signed wheel travel of each half. Segment k
/// joins chunks k and k+1:
///   b_k = c_{k+1} - c_k,  l_k = m_{k+1} - m_k,  a_k = s_k (cos, sin)((psi_k + psi_{k+1}) / 2)
/// Turn gate: a segment whose heading change between the chunk centres (the larger of the net
/// change and the total absolute variation of the steps, from heading_variation_rad), or the
/// variation inside either chunk, exceeds align_yaw_max_turn_rad is skipped (a_k = 0,
/// n_skipped_turns()).
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
/// Turns with wheel data (align_yaw_wheel_turns, gnss.md section 5.1): a segment the turn gate
/// rejects, when every step into the fixes of both chunks came with wheel data and turned at most
/// align_yaw_max_turn_rad, is used with the direction a_k = unit(g_k) of the antenna displacement
/// in w between the two chunk means, g_k = Q_{k+1} - Q_k + mo_{k+1} - mo_k: Q is the path of the
/// odometry frame o (T_b_o) integrated from the wheel travel along o's unwrapped heading (exact
/// for constant speed and yaw rate between fixes), mo the mean horizontal R_w_b (p_b_g - p_b_o).
/// The lever arm is inside g_k, so the segment adds nothing to L. Its direction error adds to the
/// numerator of sigma_yaw per chunk: wheel slip and skid (align_yaw_wheel_slip_frac of the
/// weighted path along and across, fully correlated inside the chunk, triangle bound over the two
/// segments sharing it) and the unknown speed profile between fixes ((s dpsi)^2 / 12 across each
/// step, projected on both segments with their exact weights) (n_wheel_turn_segments()). A
/// weighted path below align_yaw_min_wheel_travel_m is stationary.
/// Wheel/GNSS consistency window: with d_k the wheel-predicted displacement of a segment (the
/// weighted wheel travel between the centroids, or |g_k|), the segments since the last mismatch
/// are also summed; a mismatch is |r - sum d_k| > align_yaw_reverse_nsigma x their noise +
/// align_yaw_wheel_slip_frac x sum d_k (n_wheel_mismatches()). After a mismatch the older segments
/// are suspect: no transform ("track not coherent") until the recent segments agree with the
/// wheels over align_yaw_wheel_restart_m of wheel travel; then they replace all segments
/// (n_wheel_restarts()). A segment without wheel data breaks the window (it starts again at the
/// next segment; a pending restart stays pending). Agreement implies r >= (1 - slip) sum d_k -
/// n sigma, so with wheel data
/// the coherence test below also holds against the wheel travel.
/// Root-cause study (gnss_fusion_study.md section 16.13): a run of at most
/// align_yaw_wheel_max_gap_fixes fixes without an odometer reading is bridged by the next reading
/// (its travel lands on that fix; the wheel path is not bridged). A stationary segment is extended
/// over up to align_yaw_merge_max_chunks more chunks until it has align_yaw_min_wheel_travel_m of
/// travel, so a slow robot's travel adds up; its direction is the path of o between the chunk
/// means (front end where observed, else the wheel path; extended only while one will exist,
/// skipped if it ends without), because the mean heading of its end chunks does not represent a
/// long span. With
/// align_yaw_frontend_travel, when every step from the first fix of the earlier chunk to the last
/// fix of the later one was observed along the travel axis by the LiDAR, a front-end displacement
/// of o between the chunk means below align_yaw_min_wheel_travel_m makes the segment stationary
/// (wheels turning on a still body, a parked robot), and without wheel data the sign of that
/// displacement along a_k is the direction of travel.
/// A transform is returned once confident: r >= the minimum track (align_yaw_min_track_wheel_m
/// when every segment used has its direction from wheel data, else align_yaw_min_track_m),
/// r >= align_yaw_min_coherence * sum |b_k| over the segments used, no wheel/GNSS mismatch
/// pending, and sigma_yaw <= align_yaw_max_sigma_rad.
/// Not handled (gnss.md sections 5 and 5.1; causes and tolerances in gnss_fusion_study.md
/// section 16.13): where the LiDAR does not observe the travel axis, without wheel data net
/// backward motion before the estimate is confident gives yaw + 180 deg and a parked robot whose
/// GNSS wanders can align on noise, and with wheel data wheels turning on a still body with GNSS
/// wandering by 0.2 m per fix can align on noise; after wheels spun on a still body the alignment
/// waits for align_yaw_wheel_restart_m of agreeing travel; an odometer that over-reads by 2x or
/// more never aligns (n_wheel_mismatches() grows); a skid that persists through a long one-way
/// turn is not in the sigma.
/// Without heading_variation_rad readings, opposite turns hidden in one step (a GNSS outage over
/// an S-jog) cancel in the net heading change and pass the turn gate. Coloured GNSS error is not
/// in sigma_yaw; the minimum tracks and the coherence tests only bound its tails.
/// Translation: p_e_w = mean(p_e_g) - R_e_w mean(p_w_g) over all fixes, as GnssAligner.
///
/// O(1) memory, no allocation. Ownership/threading: single-threaded; owned by
/// pipeline::Estimator (backend thread) when gnss.align_yaw_from_track is true.
class GnssTrackYawAligner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Stores the align_yaw_* thresholds (validated by load_config). Errors: none. Hot path: no.
  /// Tests: test_gnss_track_yaw.
  explicit GnssTrackYawAligner(const GnssConfig& cfg);
  /// As above, with the odometry frame `T_b_o` (frames.T_b_o): the wheel odometer is the forward
  /// speed of frame o, so the wheel path (align_yaw_wheel_turns) is integrated for o along its
  /// heading and the antenna lever arm from o is `p_b_g - p_b_o`. The one-argument form uses
  /// identity (o = b). Errors: none. Hot path: no.
  GnssTrackYawAligner(const GnssConfig& cfg, const common::Pose& T_b_o);

  /// Adds one fix and returns the current `T_e_w` estimate when available.
  /// Frames: `p_e_g` antenna in e, `cov_e` its covariance in e (m^2; each of its two horizontal
  ///   diagonal entries must be > 0, and their mean, floored at align_yaw_min_var_m2, is used);
  ///   `T_w_b` body pose in w at the fix time; `p_b_g` lever arm in b. Result: T_e_w maps w to e.
  /// `wheel_odometer_m`: absolute signed wheel odometer reading at the fix (m, increasing when
  ///   driving forward), or NaN when no wheel data covers the interval since the previous fix.
  ///   The travel of a fix is the difference to the reading of the last fix added here, so the
  ///   travel during fixes rejected before the aligner is carried to the next one.
  /// `heading_variation_rad`: absolute reading of the total heading variation (sum of |heading
  ///   change| of the front end, rad, non-decreasing) at the fix, or NaN when unknown; the step
  ///   variation is the difference to the last fix added here, at least the net heading change
  ///   (the net change alone when NaN). It lets the turn gate see turns hidden between fixes.
  /// `travel_unobserved_scans`: absolute count of the front-end scans whose matching did not
  ///   observe the travel axis (fewer than align_yaw_frontend_min_info constraints along it, or
  ///   no match; non-decreasing), or NaN when unknown; a step is observed when the count did not
  ///   change since the last fix added here (align_yaw_frontend_travel).
  /// Errors (Result::error), first failure returned:
  ///   1. "non-finite input" (wheel_odometer_m, heading_variation_rad and
  ///      travel_unobserved_scans may be NaN, not infinite; fix NOT added);
  ///   2. "non-positive covariance" when cov_e(0,0) <= 0 or cov_e(1,1) <= 0 (fix NOT added);
  ///   3. "heading undefined" when the travel axis x_o is within about 1e-6 rad of vertical
  ///      (fix NOT added);
  ///   4. "not enough track" until two chunks are complete and the closed form has a solution
  ///      with r > 0 (fix IS added);
  ///   5. "track too short" while r < the minimum track (align_yaw_min_track_wheel_m when every
  ///      segment used has its direction from wheel data, else align_yaw_min_track_m; fix IS
  ///      added);
  ///   6. "track not coherent" while r < align_yaw_min_coherence * sum |b_k|, or a wheel/GNSS
  ///      mismatch is pending (fix IS added);
  ///   7. "yaw sigma too large" while sigma_yaw > align_yaw_max_sigma_rad (fix IS added).
  /// The yaw is re-estimated when a chunk completes; between chunks the last yaw is returned with
  /// the translation updated by the new fix.
  /// Hot path: no (per fix); no allocation, no logging. Tests: test_gnss_track_yaw.
  common::Result<common::Pose> update(
      const common::Vec3& p_e_g,
      const common::Mat3& cov_e,
      const common::Pose& T_w_b,
      const common::Vec3& p_b_g,
      double wheel_odometer_m = std::numeric_limits<double>::quiet_NaN(),
      double heading_variation_rad = std::numeric_limits<double>::quiet_NaN(),
      double travel_unobserved_scans = std::numeric_limits<double>::quiet_NaN());

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
  /// Turn-gated segments used with their direction from the wheel path (align_yaw_wheel_turns).
  [[nodiscard]] int n_wheel_turn_segments() const { return n_wheel_turns_; }
  /// Chunks at which the GNSS track since the last mismatch disagreed with the wheel travel by
  /// more than its noise and the slip fraction (wheels turning on a still body, an odometer
  /// with a wrong scale, slipping wheels; gnss.md section 5.1).
  [[nodiscard]] int n_wheel_mismatches() const { return n_wheel_mismatches_; }
  /// Times all segments were replaced by those since the last mismatch, because the segments
  /// as a whole had fallen behind the wheel travel while the recent ones agree with it (the robot
  /// drives again after its wheels turned on a still body).
  [[nodiscard]] int n_wheel_restarts() const { return n_wheel_restarts_; }

 private:
  struct Chunk {
    Eigen::Vector2d c = Eigen::Vector2d::Zero();  // m, mean position (shifted by ref_e_)
    double psi = 0.0;                             // rad, mean unwrapped heading of the travel axis
    double span = 0.0;             // rad, heading variation inside the chunk (>= max - min)
    double turn_first_rad = 0.0;   // rad, heading variation of the steps into fixes 1.. of half 1
    double turn_second_rad = 0.0;  // rad, heading variation of the steps into half 2
    double turn_into_rad = 0.0;    // rad, heading variation of the step into the first fix
    double var = 0.0;              // m^2, per-axis variance of c
    double wheel_first_m = 0.0;    // m, signed wheel travel, first half
    double wheel_second_m = 0.0;   // m, signed wheel travel, second half
    bool wheel_first_ok = false;   // every fix of the first half had wheel data
    bool wheel_second_ok = false;
    // Wheel path (align_yaw_wheel_turns): mean body position Q of the wheel path, w (m); the
    // path weighted as it enters Q_{k+1} - Q_k as the later chunk (fall) or the earlier one
    // (rise), m; the same for the per-step speed-profile variance, m^2.
    Eigen::Vector2d q = Eigen::Vector2d::Zero();
    Eigen::Vector2d mo = Eigen::Vector2d::Zero();  // m, mean horizontal lever arm from o, w
    // m^2, sums of w_rise w_fall, w_rise^2 and w_fall^2 times the profile variance u u^T
    Eigen::Matrix2d prof_cross_m2 = Eigen::Matrix2d::Zero();
    double path_rise_m = 0.0;
    double path_fall_m = 0.0;
    Eigen::Matrix2d prof_rise_m2 = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d prof_fall_m2 = Eigen::Matrix2d::Zero();
    double wtravel_rise_m = 0.0;  // m, signed wheel travel weighted as for path_rise_m
    double wtravel_fall_m = 0.0;  // m, the same as for path_fall_m
    bool path_rise_ok = false;    // every step into fixes 1.. of this chunk had wheel data
    bool path_fall_ok = false;    // every step into a fix of this chunk had wheel data
    Eigen::Vector2d po = Eigen::Vector2d::Zero();  // m, mean front-end position of o, w
    bool fe_ok = false;  // every step into a fix of this chunk observed the travel axis
  };
  // A stationary segment being extended (align_yaw_merge_max_chunks): its totals from the
  // centre of prev_ to the centre of near_, the last chunk it was extended over.
  struct Carry {
    int chunks = 0;         // chunks it was extended over (0: none)
    double travel_m = 0.0;  // m, signed wheel travel
    bool wheel_ok = true;   // every step had wheel data
    bool fe_ok = true;      // every step observed the travel axis
    bool path_ok = true;    // every step of the chunks extended over is in the wheel path
  };
  // Sums over a run of consecutive segments (gnss.md sections 5 and 5.1): Z = sum conj(a) b and
  // L = sum conj(a) l (real part: dot, imaginary: cross), the path and wheel lengths, the noise
  // of Z and the last coefficients.
  struct Segments {
    double dot_m = 0.0;             // m, sum a . b
    double cross_m = 0.0;           // m, sum a x b
    double al_dot_m = 0.0;          // m, sum a . l
    double al_cross_m = 0.0;        // m, sum a x l
    double abs_b_m = 0.0;           // m, sum |b| over the segments used
    double wheel_disp_m = 0.0;      // m, wheel-predicted displacement of the wheel segments used
    double noise_q_m2 = 0.0;        // m^2, sum var_k |a_{k-1} - a_k|^2 over closed coefficients
    double wheel_dir_var_m2 = 0.0;  // m^2, wheel-path direction error of the closed chunks
    Eigen::Vector2d prev_a = Eigen::Vector2d::Zero();  // last segment direction (0 before one)
    Eigen::Vector2d prev_a_wheel_path = Eigen::Vector2d::Zero();  // last a if from the wheel path
    int n_no_wheel = 0;  // segments with the forward-driving assumption
  };
  // Closed-form solution of a run of segments: r (m), yaw (rad), |Z|^2 (m^2) and the variance
  // across Z (m^2), with the pending terms of the last chunk prev_.
  struct Solution {
    double r_m = -1.0;
    double yaw_rad = 0.0;
    double z2_m2 = 0.0;
    double var_m2 = 0.0;
  };
  void close_chunk();
  // Adds segment (b, l, a) to `s`; prev_ is still the earlier chunk of the segment.
  void add_segment(Segments& s,
                   const Eigen::Vector2d& b,
                   const Eigen::Vector2d& l,
                   const Eigen::Vector2d& a,
                   const Eigen::Vector2d& a_wheel_path,
                   double wheel_disp_m,
                   bool no_wheel) const;
  [[nodiscard]] Solution solve(const Segments& s) const;
  // The minimum track: the wheel one when every segment used has its direction from wheel data.
  [[nodiscard]] double min_track() const {
    return all_.n_no_wheel == 0 ? min_track_wheel_m_ : min_track_m_;
  }
  [[nodiscard]] bool confident() const {
    return have_yaw_ && !restart_pending_ && track_m_ >= min_track() &&
           track_m_ >= min_coherence_ * all_.abs_b_m && yaw_sigma_rad_ <= max_sigma_rad_;
  }
  [[nodiscard]] double chunk_dir_var_m2(const Chunk& ch,
                                        const Eigen::Vector2d& c_before,
                                        const Eigen::Vector2d& c_after) const;

  int chunk_fixes_;            // fixes per chunk, from GnssConfig
  int max_gap_fixes_;          // fixes, from GnssConfig
  int merge_max_chunks_;       // chunks, from GnssConfig
  bool frontend_travel_;       // from GnssConfig
  double max_sigma_rad_;       // rad, from GnssConfig
  double min_track_m_;         // m, from GnssConfig
  double min_coherence_;       // ratio, from GnssConfig
  double reverse_nsigma_;      // sigmas, from GnssConfig
  double max_turn_rad_;        // rad, from GnssConfig
  double min_var_m2_;          // m^2, from GnssConfig
  double min_wheel_travel_m_;  // m, from GnssConfig
  bool wheel_turns_;           // from GnssConfig
  double slip_frac_;           // fraction of the path, from GnssConfig
  double min_track_wheel_m_;   // m, from GnssConfig
  double wheel_restart_m_;     // m, from GnssConfig
  common::Pose T_b_o_;         // odometry frame o in b (frames.T_b_o): travel axis x_o, p_b_o
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
  std::array<double, 2> chunk_turn_rad_{0.0, 0.0};         // rad, heading variation per half
  double chunk_turn_into_rad_ = 0.0;                       // rad, step into the chunk's first fix
  double last_heading_variation_rad_ = std::numeric_limits<double>::quiet_NaN();  // at last fix
  double chunk_sum_var_m2_ = 0.0;                  // m^2, floored per-fix variances
  std::array<double, 2> chunk_wheel_m_{0.0, 0.0};  // m, signed wheel travel per half
  double last_odometer_m_ = std::numeric_limits<double>::quiet_NaN();  // m, last reading
  std::array<bool, 2> chunk_wheel_ok_{true, true};  // every fix of the half had wheel
  // Fixes without an odometer reading since the last one, and where that run started (chunk
  // sequence number and half) (align_yaw_wheel_max_gap_fixes).
  int odo_missing_ = 0;
  int gap_seq_ = 0;
  std::size_t gap_half_ = 0;
  int chunk_seq_ = 0;  // chunks closed so far
  // Front end (align_yaw_frontend_travel): mean position of o in the current chunk (w, shifted
  // by ref_w_), whether every step into it observed the travel axis, and the last count.
  Eigen::Vector2d chunk_sum_po_ = Eigen::Vector2d::Zero();  // m
  bool chunk_fe_ok_ = true;
  double last_unobserved_scans_ = std::numeric_limits<double>::quiet_NaN();
  // Stationary segment being extended: its totals and the last chunk it was extended over.
  Carry carry_;
  Chunk near_;
  // Wheel path of the body in w (align_yaw_wheel_turns), relative to its start, and the
  // current chunk's sums (see Chunk).
  Eigen::Vector2d path_q_ = Eigen::Vector2d::Zero();               // m
  Eigen::Vector2d chunk_sum_q_ = Eigen::Vector2d::Zero();          // m
  Eigen::Vector2d chunk_sum_lo_ = Eigen::Vector2d::Zero();         // m, R_w_b (p_b_g - p_b_o)
  Eigen::Matrix2d chunk_prof_cross_m2_ = Eigen::Matrix2d::Zero();  // m^2
  double chunk_path_rise_m_ = 0.0;                                 // m
  double chunk_path_fall_m_ = 0.0;                                 // m
  Eigen::Matrix2d chunk_prof_rise_m2_ = Eigen::Matrix2d::Zero();   // m^2
  Eigen::Matrix2d chunk_prof_fall_m2_ = Eigen::Matrix2d::Zero();   // m^2
  double chunk_wtravel_rise_m_ = 0.0;                              // m
  double chunk_wtravel_fall_m_ = 0.0;                              // m
  bool chunk_path_rise_ok_ = true;
  bool chunk_path_fall_ok_ = true;
  // Previous complete chunk and the last segment direction.
  bool have_prev_ = false;
  Chunk prev_;
  // All segments (since the start or the last restart), and those since the last wheel/GNSS
  // mismatch (gnss.md section 5.1).
  Segments all_;
  Segments recent_;
  bool restart_pending_ = false;  // a mismatch since all_ started: wait for recent_ to replace it
  int n_reversals_ = 0;
  int n_skipped_turns_ = 0;
  int n_no_wheel_segments_ = 0;
  int n_stationary_ = 0;
  int n_wheel_turns_ = 0;
  int n_wheel_restarts_ = 0;
  int n_wheel_mismatches_ = 0;
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
