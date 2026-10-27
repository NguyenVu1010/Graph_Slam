# Backend: GTSAM factors, charts and marginals

Implemented in `core/backend/` (`mss::backend::Graph`, internals in `mss::backend::detail`,
`core/backend/src/`). Tested by `tests/unit/backend/test_backend_*.cpp` (B-01 to B-04 of
`docs/Test_CI_CD.md`) and `tests/unit/test_backend_api.cpp`. Conventions:
`docs/conventions.md` and `docs/derivations/lie_conventions.md` ("Library orders"). The IMU
factor replaces the `CombinedImuFactor` of ADR-004 (ADR-015).

## 1. Charts

`graph.cpp` refuses to compile unless GTSAM defines `GTSAM_POSE3_EXPMAP`, so

    Pose3::retract(xi_g) = T * Expmap(xi_g),   xi_g = [dphi, drho]
    R <- R Exp(dphi),   p <- p + R drho + O(|xi|^2)

This is the mss right perturbation `T <- T * Exp(xi_m)` with `xi_m = [drho, dphi] = P xi_g`,
`P = [0 I; I 0]` (`P = P^T = P^-1`). `Vector3` (velocity, m/s) is additive.
`imuBias::ConstantBias` is additive with tangent `[dba, dbg]` (accelerometer first).

Permutation rules at the boundary (code: `core/backend/src/conversions.hpp`):

    vectors       xi_g = P xi_m
    covariances   Sigma_g = P Sigma_m P^T
    Jacobian cols J_g = J_m P          (a residual differentiated w.r.t. xi_g = P xi_m)

Rows need no permutation when a factor whitens its own residual: the residual order of a
custom factor is internal to it, and only `e^T Sigma^-1 e` reaches the optimiser. A
`BetweenFactor<Pose3>` residual is GTSAM-ordered `[phi, rho]`, so its noise is
`P Sigma_m P^T` (checked by `BackendConversions.CovariancePermutationMatchesGtsamWhitening`).

## 2. Priors on keyframe 0 (and the IMU-gap re-anchor, §9)

Centred on the keyframe guess `x0`:

- `PriorFactor<Pose3>(X(0), x0.T_w_b)`, error `Logmap(x0^-1 T)` in `[phi, rho]`, sigmas
  `[prior_rot_sigma_rad x3, prior_pos_sigma_m x3]`. In mss order this is the header's
  `diag(prior_pos^2 x3, prior_rot^2 x3)` on the `[rho, phi]` tangent of `T_w_b`.
- `PriorFactor<Vector3>(V(0), x0.v_w_b)`, sigma `prior_vel_sigma_mps x3`.
- `PriorFactor<ConstantBias>(B(0), x0.bias)`, sigmas
  `[prior_bias_acc_sigma_mps2 x3, prior_bias_gyro_sigma_radps x3]`.

## 3. IMU factor (`detail::ImuPreintFactor`)

Variables `X_i = (R_i, p_i)`, `V_i = v_i`, `X_j`, `V_j`, `B_i = (ba_i, bg_i)`. Constants: the
delta of `core/imu` (`dR, dv, dp, dt_s = T`, `J_*`, `bias_lin`, `cov`) and
`g_w = [0, 0, -gravity_mps2]`. First-order bias correction (`imu_preintegration.md` §5) with
`db = B_i - bias_lin`, `theta = J_dR_bg dbg`:

    dR' = dR Exp(theta)
    dv' = dv + J_dv_ba dba + J_dv_bg dbg
    dp' = dp + J_dp_ba dba + J_dp_bg dbg

With

    A = R_i^T (p_j - p_i - v_i T - 0.5 g_w T^2)
    C = R_i^T (v_j - v_i - g_w T)

the 9-vector residual, order `[phi, p, v]` (the order of `delta.cov`), is

    r_phi = Log(dR'^T R_i^T R_j)
    r_p   = dR'^T (A - dp')
    r_v   = dR'^T (C - dv')

It is zero exactly when `X_j, V_j = Preintegrator::predict(X_i, V_i, B_i)`, and equals GTSAM
`NavState::localCoordinates` of `state_j` with respect to the prediction (rotation, then
position and velocity rotated into the predicted body frame), i.e. the delta chart of
`imu_preintegration.md` §1. Noise: `noiseModel::Gaussian::Covariance(delta.cov)`, no
permutation.

Jacobians (GTSAM charts, pose columns `[dphi, drho]`, bias columns `[dba, dbg]`),
`Jri = Jr^-1(r_phi)`. They follow from `Exp(phi)^T u ~ u + skew(u) phi`,
`Exp(-u) E = E Exp(-E^T u)` and `Exp(theta + d) ~ Exp(theta) Exp(Jr(theta) d)`:

    X_i:  d r_phi = [ -Jri R_j^T R_i ,  0      ]
          d r_p   = [ dR'^T skew(A)  , -dR'^T  ]
          d r_v   = [ dR'^T skew(C)  ,  0      ]
    V_i:  [ 0 ;  -dR'^T R_i^T T ;  -dR'^T R_i^T ]
    X_j:  [ Jri , 0 ;  0 , dR'^T R_i^T R_j ;  0 , 0 ]
    V_j:  [ 0 ;  0 ;  dR'^T R_i^T ]
    B_i:  d r_phi = [ 0              , -Jri Exp(r_phi)^T Jr(theta) J_dR_bg            ]
          d r_p   = [ -dR'^T J_dp_ba , skew(r_p) Jr(theta) J_dR_bg - dR'^T J_dp_bg ]
          d r_v   = [ -dR'^T J_dv_ba , skew(r_v) Jr(theta) J_dR_bg - dR'^T J_dv_bg ]

Two representative lines. `X_i` rotation: `R_i^T <- Exp(-dphi) R_i^T`, so
`A <- A + skew(A) dphi`, and `dR'^T R_i^T R_j <- dR'^T R_i^T R_j Exp(-(R_i^T R_j)^T dphi)`,
whose Log moves by `-Jri R_j^T R_i dphi`. `B_i` gyro bias: `dR'^T <- Exp(-u) dR'^T` with
`u = Jr(theta) J_dR_bg dbg`; then `r_p <- r_p - skew(u) r_p = r_p + skew(r_p) u`, and
`r_phi` moves by `-Jri Exp(r_phi)^T u`.

## 4. Bias random walk

`BetweenFactor<ConstantBias>(B_i, B_j, zero)`, sigmas
`[acc_random_walk sqrt(T) x3, gyro_random_walk sqrt(T) x3]` (m/s^2, rad/s). The constructor
rejects a random walk `<= 0` with `std::invalid_argument`: a zero sigma is a hard constraint
that the iSAM2 Cholesky factorisation cannot handle.

## 5. LiDAR and loop factors

`BetweenFactor<Pose3>(X_i, X_j, Z = T_bi_bj)`, GTSAM error `Logmap(Z^-1 T_i^-1 T_j)` in
`[phi, rho]` = `P` times the header's mss residual; noise
`noiseModel::Gaussian::Covariance(P cov P^T)`. GTSAM differentiates only `Between()`, not
the `Logmap` of the error, so its Jacobian omits `Jr^-1(e)`: it is exact at zero error and
approximate (relative error of order `|e|`) elsewhere. This is GTSAM's standard behaviour and
is the same for iSAM2 and the batch optimiser (B-04); B-01 checks it at zero error.
The approximation changes the Gauss-Newton step, so its fixed point differs from the true MAP
estimate only by terms of relative order `|e|^2`: negligible for consistent LiDAR edges
(residuals at sigma level) and for loops, whose large residuals the Cauchy kernel
down-weights. If LiDAR covariances become very tight, two exact alternatives exist: build
GTSAM with `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR`, or use a custom between factor that
reuses `odom::relative_pose_residual` (exact `Jr^-1`) the way `OdomBetweenFactor` does.

Loop: the same factor with `noiseModel::Robust::Create(mEstimator::Cauchy::Create(k),
Gaussian::Covariance(P cov P^T))`. GTSAM reweights on the whitened norm `|e|` with weight
`w = 1 / (1 + (|e| / k)^2)`. `k = kLoopCauchyK = sqrt(chi2_6(0.95)) = sqrt(12.5916) = 3.5485`:
a loop inside the 95 % gate of its own covariance keeps a weight above 0.5. A
`backend.loop_cauchy_k` YAML key comes with the loop module.

Degeneracy report: diagnostics only (one debug log line per degenerate keyframe, with the
running count); the factor is unchanged.

## 6. GNSS position factor (`detail::GnssPositionFactor`)

Variable `X_k = (R_w_b, p_w_b)`; constants `T_e_w = (R_e_w, p_e_w)` and the lever arm
`p_b_g`. With the header sign (opposite of `gnss::detail::gnss_residual`):

    r = R_e_w (R_w_b p_b_g + p_w_b) + p_e_w - p_e_g          (3x1, m)
    d r / [dphi, drho] = [ -R_e_w R_w_b skew(p_b_g) ,  R_e_w R_w_b ]

Noise `noiseModel::Gaussian::Covariance(m.cov)`.

## 7. Wheel-odometry factor (`detail::OdomBetweenFactor`)

Variables `X_i, X_j`. `r_m, J_m_i, J_m_j` come from
`odom::relative_pose_residual(T_i, T_j, odom.T_b0_b1)` (mss order, right perturbation).
`odom.cov` is singular by design. Eigen split (`SelfAdjointEigenSolver`, ascending)
`cov = U diag(lambda) U^T`; keep the `n` columns with `lambda > kOdomRankRelTol * lambda_max`,
`kOdomRankRelTol = 1e-9` (a numerical rank tolerance, not a tunable). Whitening

    W = diag(lambda_kept)^-1/2 U_kept^T                    (n x 6)
    residual  W r_m,   Jacobians  W J_m_i P,  W J_m_j P,   noise Unit(n)

so `|W r_m|^2 = r_m^T cov^+ r_m` and the unobserved directions carry no information.
`n = 0` (all-zero covariance, e.g. `t0 == t1`) or a non-finite input is rejected.

## 8. Marginals

`ISAM2::marginalCovariance(X(k))` is `[phi, rho]` at the current estimate, right
perturbation; `Graph::marginal_cov` returns `P Sigma P^T`.

## 9. Failure handling and IMU-gap re-anchor

Failure. `optimize()` inserts only the variables referenced by pending factors (an
unconstrained keyframe is never inserted; its state is reported from the guess with a NaN
covariance). When `ISAM2::update` throws (e.g. indeterminant system) or the estimate is not
finite, the iSAM2 instance may be half-updated, so it is rebuilt from the committed factors
at the last good estimate; pending factors are dropped, pending keyframe-0 priors and the
keyframe guesses are kept, and `optimize()` returns `"optimization failed: ..."`. If the
rebuild itself throws, iSAM2 restarts empty: the last estimate becomes the keyframe guesses
and every committed factor is queued again for the next `optimize()` (logged as an error).

Pose re-anchor after a failure. Dropping the pending factors also drops the edges `(k-1, k)`
that tied the keyframes added since the last success to the anchored graph, and the pipeline
never adds them again. Every keyframe whose pose is not in iSAM2 after a failure is marked
orphaned. In each `optimize()`, the keys of the pending factors are split into connected
components (by shared factors); a component that holds no key already in iSAM2 and not
`X(0)`, and contains an orphaned keyframe, gets a `PriorFactor<Pose3>` on its lowest pose
key (the oldest keyframe of the island), centred on that keyframe's guess, with the
first-keyframe sigmas (§2), and one warning line. A keyframe stops being orphaned once its
pose is in iSAM2. Components without an orphaned keyframe are not anchored, so an
under-constrained new keyframe (e.g. a GNSS fix alone) still fails as before.

Non-finite guess. `add_keyframe` replaces a non-finite `x0` pose, velocity or bias by the
previous keyframe's guess (identity / zero for keyframe 0) with one warning line; the
keyframe is still added, so KeyIds stay dense, and no non-finite value reaches GTSAM.

Re-anchor. If an IMU factor is rejected in the middle of the chain (e.g. `1 -> 2`), the next
accepted factor `2 -> 3` would bring in `V(2)` and `B(2)` with no link to keyframe 0, so the
system would stay indeterminant. When `add_imu_factor(i, j)` is accepted with `i > 0` and no
accepted IMU factor ends at `i`, priors on `V(i)` and `B(i)` with the first-keyframe sigmas
(§2), centred on the guess of keyframe `i`, are added with it.

## 10. Checks

| Formula | Test |
| --- | --- |
| §1 permutation of covariances | `BackendConversions.CovariancePermutationMatchesGtsamWhitening`, `BackendConversions.PermuteCovIsInvolution` |
| §1 pose conversion | `BackendConversions.PoseRoundTrip` |
| §3 IMU Jacobians (numerical, < 1e-6) and full `delta.cov` noise | B-01 `BackendFactors.B01_ImuFactorJacobians` |
| §3 zero residual at the prediction | B-01 `BackendFactors.B01_ImuFactorZeroAtPrediction` |
| §6 GNSS Jacobian and sign | B-01 `BackendFactors.B01_GnssFactorJacobians`, `BackendFactors.B01_GnssFactorZeroWhenConsistent` |
| §7 rank-3 odom whitening and Jacobians | B-01 `BackendFactors.B01_OdomFactorRotatedSingularCov`, `BackendFactors.B01_OdomFactorZeroCovRejected` |
| §5 permuted noise and Cauchy wrap (at zero error; `sqrt(w)` scaling at 0.1) | B-01 `BackendFactors.B01_BetweenAndRobustLoopJacobians` |
| §2, §8 priors and marginal order | `BackendGraph.MarginalCovIsMssOrder` |
| §5 pose graph | B-02 `BackendGraph.B02_SquareLoopNoiseFreeConvergesExactly`, `BackendGraph.B02_SquareLoopNoisyWithinMarginalBounds` |
| §3, §4 IMU chain and time check | `BackendGraph.ImuChainRecoversTrajectory`, `BackendGraph.ImuTimeCheckRejectsMismatch` |
| §6 in the graph | `BackendGraph.GnssFactorPullsPosition` |
| §9 pose re-anchor, non-finite guess, duplicate IMU factor | `BackendGraph.FailedOptimizeDoesNotStrandLaterKeyframes`, `BackendGraph.AddKeyframeNonFiniteGuessIsSanitised`, `BackendGraph.DuplicateImuFactorRejected` |
| §9 | `BackendGraph.ImuGapReanchorsVelocityAndBias`, `BackendGraph.OptimizeFailureDropsPendingFactorsKeepsKeyframes`, `BackendGraph.LatestKeyframeWithoutFactorsReturnsGuessAndNanCov` |
| input validation | `BackendGraph.RejectsBadCovariances`, `BackendApi.ConstructorRejectsInvalidConfig` |
| §5 robust loop | B-03 `BackendIsam.B03_OutlierLoopDragsLessThanTenCm`, `BackendIsam.B03_ControlNonRobustOutlierDragsMoreThanTenCm` |
| iSAM2 vs batch LM | B-04 `BackendIsam.B04_IsamMatchesBatchLevenbergMarquardt` |
| determinism (single-thread TBB in `deterministic` mode: ADR-018), diagnostics | `BackendIsam.RepeatedOptimizeIsStableAndDeterministic`, `BackendDeterminism.SingleThreadLimitIsProcessLifetime`, `BackendDeterminism.SingleThreadRunsAreBitIdentical`, `BackendDeterminism.MultiThreadRunsReported`, `BackendIsam.DegeneracyReportIsDiagnosticOnly` |
