# IMU preintegration (on-manifold, GTSAM ManifoldPreintegration scheme)

Implemented in `core/imu/src/preintegrator.cpp` (`mss::imu::Preintegrator`).
Tested by `tests/unit/test_imu_preintegrator.cpp`, `tests/unit/test_imu_covariance.cpp`
and `tests/unit/backend/test_imu_preint_gtsam.cpp` (I-01..I-05).

Notation: for sample `k >= 1` of an interval, `a = s_k.acc_b - ba_lin`,
`w = s_k.gyro_b - bg_lin`, `dt = seconds_between(t_{k-1}, t_k)`, `dR_k = Exp(w dt)`,
`R = dR` before the update, `x^ = skew(x)`, `Jr` the SO(3) right Jacobian
(`docs/derivations/lie_conventions.md`).

## 1. Variables and conventions

The delta is `(dR = R_bi_bj, dp, dv)`, with `dp` and `dv` expressed in `b_i`, linearised
at the bias `bias_lin = (bg_lin, ba_lin)`. Gravity is not in the deltas; `predict()` adds
it (§5).

Perturbation is right on SO(3). The 9x9 covariance is expressed in the chart

    dR_true = dR * Exp(d_phi)
    dp_true = dp + dR * d_p
    dv_true = dv + dR * d_v

with block order `[phi, p, v]`. This is exactly GTSAM `NavState::retract` (local
coordinates of a `NavState`), which is why the backend needs no permutation. It is NOT the
additive `b_i` chart of Forster et al. (where `dp_true = dp + d_p`); the two differ by the
rotation `dR` on the p and v blocks.

Bias Jacobians:

- rotation, right perturbation: `dR(b + db) ~ dR(b) * Exp(J_dR_bg * dbg)`;
- velocity and position, additive: `dv(b + db) ~ dv(b) + J_dv_ba dba + J_dv_bg dbg`,
  the same for `dp`.

`dR` does not depend on `ba`, so there is no `J_dR_ba`.

## 2. Integration rule (mean)

The first sample after `reset()` sets `t_first = t_prev = t_0` and integrates nothing.
Each later sample `k` applies, in this order (all right-hand sides use the values before
this sample):

    dp   <- dp + dv dt + 0.5 R a dt^2
    dv   <- dv + R a dt
    dR   <- R dR_k
    dt_s <- seconds_between(t_first, t_k)

This is Euler integration with the measurement of sample `k` held over `[t_{k-1}, t_k]`,
the same as GTSAM `ManifoldPreintegration::update` (via `NavState::update`, which retracts
`[w dt, dt R^T dv + 0.5 a dt^2, a dt]` with the old rotation).

## 3. Bias Jacobians

Differentiate the §2 update exactly. Write `a = a_m - ba`, `w = w_m - bg`.

Rotation. `dR' = R Exp(w dt)`. With `R(bg + d) = R Exp(J d)` (old `J = J_dR_bg`) and
`Exp((w - d) dt) = Exp(w dt) Exp(-Jr(w dt) dt d)`:

    dR'(bg + d) = R Exp(J d) Exp(w dt) Exp(-Jr dt d)
               = R Exp(w dt) Exp(dR_k^T J d) Exp(-Jr dt d)
    => J_dR_bg <- dR_k^T J_dR_bg - Jr(w dt) dt

Velocity. `dv' = dv + R a dt`. `d(R a)/dba = -R`. For `bg`:
`R Exp(J d) a ~ R a + R (J d)^ a = R a - R a^ J d`, so `d(R a)/dbg = -R a^ J`.

    J_dv_ba <- J_dv_ba - R dt
    J_dv_bg <- J_dv_bg - R a^ J_dR_bg dt

Position. `dp' = dp + dv dt + 0.5 R a dt^2`:

    J_dp_ba <- J_dp_ba + J_dv_ba dt - 0.5 R dt^2
    J_dp_bg <- J_dp_bg + J_dv_bg dt - 0.5 R a^ J_dR_bg dt^2

All use the OLD `R`, `J_dR_bg`, `J_dv_ba`, `J_dv_bg`, so the update order is:
`J_dp_*` first, then `J_dv_*`, then `J_dR_bg`. GTSAM `ManifoldPreintegration::update`
has the same recursion (`D_acc_R = -R a^`, `D_incrR_integratedOmega = Jr(w dt)`).

## 4. Covariance recursion

    Sigma <- A Sigma A^T + B (sa^2 / dt) B^T + C (sg^2 / dt) C^T

with `sa = acc_noise_density` (m/s^2/sqrt(Hz)), `sg = gyro_noise_density`
(rad/s/sqrt(Hz)), i.e. the continuous densities discretised as `density^2 / dt`, and

    A = [ dR_k^T                 0        0         ]
        [ -0.5 dR_k^T a^ dt^2    dR_k^T   dR_k^T dt ]
        [ -dR_k^T a^ dt          0        dR_k^T    ]

    B = [ 0 ; 0.5 dR_k^T dt^2 ; dR_k^T dt ]     (9x3, acts on accel noise)
    C = [ Jr(w dt) dt ; 0 ; 0 ]                 (9x3, acts on gyro noise)

Derivation. Perturb the state in the §1 chart: `R~ = R Exp(e_phi)`, `p~ = p + R e_p`,
`v~ = v + R e_v`, and the measurements: `a~ = a + n_a`, `w~ = w + n_w`.

- Rotation: `R~' = R Exp(e_phi) Exp((w + n_w) dt) = R' Exp(dR_k^T e_phi + Jr dt n_w)`,
  so `e_phi' = dR_k^T e_phi + Jr dt n_w`.
- Velocity: `v~' = v + R e_v + R Exp(e_phi)(a + n_a) dt
  = v' + R (e_v - a^ e_phi dt + n_a dt)`. Expressed in the new chart,
  `e_v' = R'^T R (...) = dR_k^T (e_v - a^ dt e_phi + dt n_a)`.
- Position: `p~' = p' + R (e_p + e_v dt - 0.5 a^ dt^2 e_phi + 0.5 dt^2 n_a)`, so
  `e_p' = dR_k^T (e_p + dt e_v - 0.5 dt^2 a^ e_phi + 0.5 dt^2 n_a)`.

Reading off the coefficients gives `A`, `B`, `C`. They depend only on the current step
(`dR_k`, `a`, `w`, `dt`), not on the old `R`. They equal GTSAM `NavState::update`'s
`F, G1, G2` (the `dt R^T v` terms of `F` cancel against `D_xiP_state`). The integration
covariance is zero and the bias random walk is not part of this 9x9 block (the backend's
bias factor owns it).

## 5. First-order bias correction and predict

With `db = x0.bias - bias_lin`:

    dR' = dR Exp(J_dR_bg dbg)
    dv' = dv + J_dv_ba dba + J_dv_bg dbg
    dp' = dp + J_dp_ba dba + J_dp_bg dbg

and, with `T = dt_s`:

    R1 = R0 dR'
    v1 = v0 + g_w T + R0 dv'
    p1 = p0 + v0 T + 0.5 g_w T^2 + R0 dp'
    t1 = x0.t + (t_last - t_first)   (integer ns)
    bias1 = x0.bias

This is GTSAM `PreintegrationBase::predict` (bias-corrected delta, `NavState::correctPIM`
without Coriolis, then `retract`).

## 6. Checks

| Formula | Test |
| --- | --- |
| §2 static input gives `dR = I` and predict returns `x0` | I-01 `Preintegrator.I01_StaticGravityOnlyPredictsNoMotion` |
| §2 constant `w` gives `dR = Exp(w dt_s)` | I-02 `Preintegrator.I02_ConstantRateMatchesExp` |
| §3 bias Jacobians, central differences, step 1e-5, relative error < 1e-6 | I-04 `Preintegrator.I04_BiasJacobiansMatchCentralDifferences` |
| §5 first-order correction | I-04 `Preintegrator.I04_PredictBiasCorrectionIsFirstOrder` |
| §4 covariance against 1000-run Monte Carlo, errors measured in the §1 chart | I-05 `PreintegratorCov.I05_MatchesMonteCarlo` |
| §2-§5 against GTSAM, 1e-9 | I-03 `ImuPreintGtsam.I03_MatchesManifoldPreintegration` |

Note on `GTSAM_TANGENT_PREINTEGRATION`. The pinned GTSAM 4.2.0 build defines this flag, so
`PreintegratedCombinedMeasurements` derives from `TangentPreintegration`, which integrates
`theta <- theta + Jr^-1(theta) w dt` and keeps its covariance and gyro-bias Jacobian in a
different chart. The full 1e-9 check (deltas, bias Jacobians, covariance, predict) is
therefore against `gtsam::ManifoldPreintegration`, which implements exactly §2-§5 and is
compiled in either build. `ImuPreintGtsam.I03_MatchesCombinedMeasurements` compares
`PreintegratedCombinedMeasurements` only on what both schemes compute identically
(single-axis gyro input: deltas, `dt_s`, accel-bias Jacobians). The backend consumes this
module's delta through its own 9x9 factor, so GTSAM's integration scheme never enters the
estimator.
