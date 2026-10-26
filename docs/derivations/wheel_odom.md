# Wheel odometry: integration, covariance and residuals

Implemented in `core/odom/src/wheel_integrator.cpp` (`mss::odom::WheelIntegrator`) and
`core/odom/src/odom_residual.cpp` (`mss/odom/odom_residual.hpp`). Conventions follow
`docs/conventions.md` and `docs/derivations/lie_conventions.md`: `T_a_b` maps points from
`b` to `a`, right perturbation `T <- T * Exp(xi)`, SE(3) tangent `xi = [rho, phi]`.
Tests: O-01 and O-02 of `docs/Test_CI_CD.md` plus the unit/component tests in section 6.

## 1. Model

The wheel-odometry frame `o` has x forward and z up. A `WheelSample` carries the forward
speed `v` (m/s, along `x_o`) and the yaw rate `omega` (rad/s, about `z_o`). The first sample
only sets the start time. Every later sample `k >= 1` uses ITS OWN reading, held constant
over

    dt_k = seconds_between(t_{k-1}, t_k)          (s, from integer ns timestamps)
    s_k  = v_k * dt_k                             (m, forward distance)
    a_k  = omega_k * dt_k                         (rad, yaw increment)

With constant `v, omega` over `dt_k` the motion is an exact circular arc, i.e. the SE(2)
increment

    Delta_k = (R(a_k), t_k),   t_k = s_k * [ sin(a)/a ,  (1 - cos a)/a ]

and the pose of `o` accumulates by right composition:

    T_o0_ok = T_o0_ok-1 * Delta_k      (x, y) += R(yaw_{k-1}) t_k,  yaw += a_k

The implementation writes `(1 - cos a)/a` as `2 sin^2(a/2)/a`, which is the same value
without the cancellation of `1 - cos a`. For `|a| < kSeriesAngleRad = 1e-4` rad it uses the
series

    sin(a)/a       ~ 1 - a^2/6
    (1 - cos a)/a  ~ a/2 - a^3/24

whose truncation error is below `a^4/120 < 1e-18` (relative). `a = 0` (straight line) is
therefore exact, and `omega = 1e-12` gives no `0/0`. The yaw is not wrapped: `R(yaw)` is
periodic, and more than one turn between keyframes is fine.

## 2. Embedding and frame change

The SE(2) pose `(x, y, yaw)` is embedded in SE(3) with `z = roll = pitch = 0`:

    T_o0_o1 = ( Rz(yaw), [x, y, 0] )

and expressed in the body frame (IMU) with the extrinsic `T_b_o` (`frames.T_b_o`):

    T_b0_b1 = T_b_o * T_o0_o1 * T_b_o^-1

## 3. Covariance

The SE(2) right tangent `[rho_x, rho_y, phi_z]` equals the SE(3) tangent restricted to the
indices `0, 1, 5` (the SE(3) `Exp` of `[rho_x, rho_y, 0, 0, 0, phi_z]` is the SE(2) `Exp`).
Per-sample noise acts on the increment `n_k = [delta_s, delta_a]`:

    Q_k = diag(sigma_v^2, sigma_omega^2) * dt_k^2

(`sigma_v = speed_sigma_mps`, `sigma_omega = yaw_rate_sigma_radps`; this is the same as
`diag(sigma_v^2, sigma_omega^2)` on the readings `(v_k, omega_k)` since `ds = dt dv`,
`da = dt domega`.) With `T_k Exp(xi_k) = T_{k-1} Exp(xi_{k-1}) Delta_k Exp(B_k n_k)` and
`Exp(xi) Delta = Delta Exp(Ad(Delta^-1) xi)`:

    xi_k = A_k xi_{k-1} + B_k n_k
    P_k  = A_k P_{k-1} A_k^T + B_k Q_k B_k^T

    A_k = Ad_SE2(Delta_k^-1),  Ad_SE2(R, t) = [ R   [t_y; -t_x] ]
                                              [ 0   1           ]
    Delta_k^-1 = (R(a)^T, -R(a)^T t_k)

    B_k = [ R(a)^T * [dt/ds, dt/da] ]    (3x2)
          [ 0             1         ]

    dt/ds = [ sin(a)/a ,  (1 - cos a)/a ]
    dt/da = s * [ (a cos a - sin a)/a^2 ,  (a sin a - 1 + cos a)/a^2 ]
          ~ s * [ -a/3 + a^3/30 ,  1/2 - a^2/8 ]        (|a| < kSeriesAngleRad)

`P` is symmetrised (`P = (P + P^T)/2`) after each sample. At `take_delta()` the 3x3 `P` is
embedded in a 6x6 `cov_o` (rows/columns 2, 3, 4 zero: z, roll, pitch of `o` are not
observed, so the covariance is singular by design) and moved to the body frame. From

    T_b_o T_o0_o1 Exp(xi) T_b_o^-1 = T_b0_b1 Exp(Ad(T_b_o) xi)

    cov = Ad(T_b_o) cov_o Ad(T_b_o)^T = transform_covariance(cov_o, T_b_o)

Systematic calibration error (wheel radius, track) and slip (ADR-011 section 3) are not
modelled; the YAML sigmas absorb them. `wheel_radius_m` and `track_m` are unused because
`WheelSample` already carries `v` and `omega`.

## 4. Relative-pose residual (O-01)

Variables: `T_w_bi`, `T_w_bj`, both with right perturbation `T <- T Exp(xi)`, `xi = [rho,
phi]`. Measurement: `Z = T_bi_bj_meas` (`OdomDelta::T_b0_b1`).

    r = Log( Z^-1 * T_w_bi^-1 * T_w_bj )        (6x1, [rho, phi]; as GTSAM BetweenFactor)

Let `E = Z^-1 T_w_bi^-1 T_w_bj`, `r = Log(E)`. Perturbing `T_w_bj`: `Log(E Exp(xi)) ~ r +
Jr^-1(r) xi`. Perturbing `T_w_bi`: `T_w_bi^-1 -> Exp(-xi) T_w_bi^-1`, so `E -> E
Exp(-Ad(T_w_bj^-1 T_w_bi) xi)`. Hence

    J_r_xi_j = Jr^-1(r)
    J_r_xi_i = -Jr^-1(r) * Ad(T_w_bj^-1 * T_w_bi)

SE(3) right Jacobian inverse: `Jr(xi) = Jl(-xi)`, so `Jr^-1(xi) = Jl^-1(-xi)` with, in
`[rho, phi]` order,

    Jl(xi)    = [ Jl(phi)   Q(rho, phi) ]
                [ 0         Jl(phi)     ]
    Jl^-1(xi) = [ Jl^-1(phi)   -Jl^-1(phi) Q(rho, phi) Jl^-1(phi) ]
                [ 0             Jl^-1(phi)                         ]

`Jl^-1(phi)` is the SO(3) left Jacobian inverse (`common::so3_jl_inv`). `Q` is Barfoot,
*State Estimation for Robotics*, eq. 7.86, with `theta = |phi|`, `P = skew(phi)`,
`X = skew(rho)`:

    Q(rho, phi) = 1/2 X
                + c1 (P X + X P + P X P)
                + c2 (P P X + X P P - 3 P X P)
                + c3 (P X P P + P P X P)

    c1 = (theta - sin theta) / theta^3
    c2 = (theta^2 + 2 cos theta - 2) / (2 theta^4)
    c3 = (2 theta - 3 sin theta + theta cos theta) / (2 theta^5)

For `theta < kSmallAngleQRad = 1e-3` rad the coefficients use their series (the next terms
are `O(theta^4)`, below `1e-12` relative):

    c1 ~ 1/6   - theta^2/120
    c2 ~ 1/24  - theta^2/720
    c3 ~ 1/120 - theta^2/2520

The SE(3) `Jr^-1` stays private to `odom_residual.cpp` until `common::lie` offers it
(contract issue 3 of the plan).

## 5. Non-holonomic residual (O-02)

A ground vehicle does not slide sideways or move vertically in its odometry frame.
Variables: `T_w_b` (right perturbation) and `v_w_b` (additive, m/s, velocity of the body
origin in `w`). Constants: `omega_b` (rad/s, angular rate in `b`, from the IMU) and
`T_b_o`. The velocity of the origin of `o`, expressed in `o`:

    v_o  = R_b_o^T ( R_w_b^T v_w_b + omega_b x p_b_o )
    r_nh = S v_o,   S = [0 1 0; 0 0 1]       (lateral and vertical velocity of o, m/s)

The residual is returned unwhitened; the backend factor chooses its sigma. With
`u = R_w_b^T v_w_b` and `Exp(phi)^T u ~ u - phi x u = u + skew(u) phi`:

    J_rnh_xi = S R_b_o^T [ 0_3   skew(R_w_b^T v_w_b) ]     (2x6, [rho, phi])
    J_rnh_v  = S R_b_o^T R_w_b^T                            (2x3)

The translation part of `T_w_b` does not enter the residual (zero columns).

## 6. Tests

| Formula | Test |
| --- | --- |
| Section 1 mean, series branch | `WheelIntegrator.ConstantArcMatchesAnalytic`, `SeriesThresholdContinuity`, `MultiTurnWraps`, `ChainsWithoutGaps`, `RejectsNonFinite`, `RejectsNonIncreasingTime`, `FirstSampleOnlySetsStart`, `EmptyBeforeAnySample` |
| Section 2 frame change | `WheelIntegrator.ExtrinsicConjugation` |
| Section 3 `A_k`, `B_k`, `Q_k` | `WheelIntegrator.SingleSampleCovKnownAnswer`, `CovMatchesNumericalPropagation`, `CovSymmetricPsdSingular`, `ZeroSigmaGivesZeroCov`, `PureRotationInPlace`, `ReverseDrivingStraight`, `LongGapHeldReading` |
| Hot path | `WheelIntegrator.HotPathNoAllocation`, `OdomResidual.NoAllocation` |
| Section 4 `r`, `J_r_xi_i`, `J_r_xi_j`, `Q` series | `OdomResidual.O01_ConsistentMotionZeroResidual`, `OdomResidual.O01_JacobiansMatchNumerical` |
| Section 5 `r_nh`, `J_rnh_xi`, `J_rnh_v` | `OdomResidual.O02_ResidualProportionalToLateralVelocity`, `OdomResidual.O02_JacobiansMatchNumerical` |
| End to end | `OdomComponent.KeyframeChainOnSyntheticTrajectory`, `OdomComponent.InjectedLateralSlipShowsInResidual` |
