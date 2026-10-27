# IMU-only observability: drift along an axis the LiDAR does not observe

Diagnosis for PR 4d, on main `94b9f16`. Question: in a corridor whose end walls are out of
LiDAR range, LiDAR + IMU only (no wheel, no GNSS), the estimate along the corridor axis
drifted 5.5 m with degeneracy off and 12 to 97 m with some degeneracy settings (PRs 4b, 4c),
10 to 40 sigma above inertial dead reckoning. Sections 1 to 7 are the diagnosis (controlled
experiments, one candidate at a time); section 8 states the mechanism; sections 9 to 13 are the fix
evaluated (PR 4d Task 2).

## Setup

Synthetic box world (`tests/component/box_world.hpp`, `pipeline.md` section 7) with its x
walls at +-1000 m, beyond `preprocess.max_range_m` (100 m), so no LiDAR return constrains
world x. Two 20 s scenes, both 3 s static, 2 s ramp, 15 s at 0.5 m/s:

- circle: the box-world default circle (R = 2 m, yaw rate 0.25 rad/s) between the y walls
  at +-5 m (PR 4b's corridor scene);
- straight: yaw rate 0.0005 rad/s, y walls at +-2.5 m (the almost straight corridor of
  `CorridorDriftCorrectedByGnss`, 15 s instead of 60 s).

IMU white noise only (no bias in the box world): accelerometer `q_a = 5.8e-3 m/s^2/sqrt(Hz)`,
gyroscope `q_g = 3.5e-4 rad/s/sqrt(Hz)` at 200 Hz (`BoxWorldParams`, equal to
`config/default.yaml`), seed 42, `deterministic: true`, GTSAM build, Release. The brief
quoted `sim/noise/default.yaml` (2.9e-3); the box world, and therefore every number here,
uses 5.8e-3. Estimates start at the first scan after `imu.init_window_s` = 2 s, so the run
spans T = 17.9 s after initialisation.

Metrics: "along-x RMS" and "end x err" are the published (front-end) estimates against the
analytic truth; "end / sigma_DR" divides the end error by the analytic dead-reckoning
sigma of section 1 at that time (0.64 m at 17 s); "backend last KF x err (sigma)" is the
backend estimate of the newest keyframe and the sigma along world x of its marginal. Per
LiDAR between factor `T_bi_bj` with covariance `cov_b`: "factor x NEES" is
`e_x^2 / var_x` of the measured translation along world x against the truth (expected 1),
"factor 6-DoF NEES" is `e^T cov_b^-1 e` with `e = boxminus(T_bi_bj, truth)` (expected 6).
"matcher x corr RMS" is the per-scan correction the scan matcher applies along world x to
the IMU prediction.

Driver: `PipelineComponent.ImuOnlyObservabilityStudy` in
`tests/component/test_imu_only_study.cpp`, skipped unless `MSS_IMU_STUDY=1` (GTSAM
build, about 2.5 min). It regenerates every table of this file into
`results/<run-id>/imu_study.md` (`MSS_IMU_STUDY_RUN_ID`, default `imu-study`) with more
columns (lateral error, Hessian information along x, tilt of the weakest Hessian
direction, keyframe velocity and bias). The controls go through the test-only seam
`detail::CoreTestHooks` (`estimator_core.hpp`): fixed-period keyframes
(`keyframe_every_n_scans`), a front end that keeps its own state at keyframes
(`frontend_keeps_own_state`), the LiDAR factor measurement replaced by the true relative
pose (`lidar_oracle`), the LiDAR factor covariance changed (`lidar_cov`), and an observer
of every match, LiDAR factor and optimised keyframe.

Settings of the LiDAR factor along world x:

| setting | matcher | factor along x |
| --- | --- | --- |
| deg off | plain Gauss-Newton (shipped `degeneracy.enabled: false`) | inverse Hessian, whatever the plane fits give |
| deg on | update projected out of flagged directions (`eig_threshold` 100) | `relax_factor` 10 (shipped): still millimetres |
| deg on, x free | as deg on | `relax_factor` 1e8 along flagged directions (the 4b "unobserved" semantics) |
| deg on, x removed | as deg on | `cov_b + (1 km)^2 d d^T`, `d` = true world x in b_j: no information along x, exactly |

## 1. Dead-reckoning reference

Per horizontal axis, integration from rest after the static window (`T_init` = 2 s):

    velocity random walk     sigma_p = q_a T^1.5 / sqrt(3)
    static-init tilt         sigma_p = 0.5 (q_a / sqrt(T_init)) T^2
    (free inertial only)     sigma_p = g (q_g / sqrt(T_init)) T^3 / 6  (+) g q_g T^2.5 / sqrt(20)

The tilt term is the accelerometer mean of the init window: its error `q_a / sqrt(T_init)`
= 4.1e-3 m/s^2 tilts the initial attitude by `4.1e-3 / g` = 0.42 mrad, which leaks gravity
along the axis for the whole run. The gyro terms (initial gyro-bias error and angle random
walk tilting the attitude) apply only when nothing else observes the attitude; with LiDAR
the relative attitude is observed and only the initial tilt remains. The reference for the
corridor is therefore VRW + init tilt: **sigma_DR = 0.64 m at 17 s** (the brief's
0.6 to 0.7 m).

Monte Carlo (200 seeds, `dead_reckoning_errors` in the driver): "free" integrates the IMU
with the pipeline's `ImuInitializer` and one `Preintegrator`; "aided" uses the true attitude
at every sample (no tilt term), which isolates VRW.

| T after init (s) | free: seed 42 err x (m) | free: MC RMS per axis (m) | aided: seed 42 err x (m) | aided: MC RMS per axis (m) | analytic VRW (m) | analytic acc tilt (m) | analytic VRW + acc tilt (m) | analytic gyro (m) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 5 | -0.137 | 0.090 | -0.033 | 0.039 | 0.037 | 0.051 | 0.063 | 0.066 |
| 10 | -0.946 | 0.496 | -0.048 | 0.111 | 0.106 | 0.205 | 0.231 | 0.472 |
| 15 | -2.436 | 1.381 | +0.001 | 0.199 | 0.195 | 0.461 | 0.501 | 1.520 |
| 17 | -3.128 | 1.873 | +0.015 | 0.238 | 0.235 | 0.593 | 0.637 | 2.188 |

The aided Monte Carlo matches the VRW formula (0.238 vs 0.235 m at 17 s). Free inertial
dead reckoning is 1.87 m at 17 s, dominated by the gyro tilt terms (2.2 m analytic, an upper
bound since the circle averages part of it); it is not the reference for a LiDAR-aided run.

## 2 to 7. Pipeline runs

Cases per setting: "baseline" (shipped pipeline); "no backend feedback" (front end keeps its
matched pose, IMU velocity and bias at keyframes instead of the backend estimate, so neither
the LiDAR factor measurements, the map insertion nor the prediction see the backend); "bias
frozen" (`backend.prior_bias_acc_sigma_mps2` 1e-4 at the true bias 0 and
`imu.acc_random_walk` 1e-7); "relin" (`backend.relinearize_threshold`, shipped 0.1); "KF
every n scans" (fixed-period keyframes instead of the 1 m / 10 deg rule; the shipped rule
gives about one keyframe per 0.7 s on the circle); "oracle measurement" (the LiDAR factor
measurement is the true relative pose, covariance unchanged); "tilt prior 0.02 deg"
(`backend.prior_rot_sigma_deg` 1 -> 0.02, i.e. 0.35 mrad, about the static-init tilt sigma
0.42 mrad); "tight kf0 priors" (rotation 0.02 deg, accelerometer bias 0.004 m/s^2 =
`q_a / sqrt(T_init)`, velocity 0.01 m/s). The last four rows of each table are section 6.

### Circle

| case | along-x RMS (m) | end x err (m) | end / sigma_DR | keyframes | matcher x corr RMS (m) | LiDAR factor x NEES (exp. 1) | LiDAR factor 6-DoF NEES (exp. 6) | backend last KF x err (sigma) (m) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| deg off, baseline | 4.498 | -4.357 | 6.2 | 24 | 0.0682 | 1.07e+03 | 1.31e+03 | -4.357 (0.135) |
| deg off, no backend feedback | 3.428 | -3.268 | 4.6 | 23 | 0.1012 | 611 | 1.01e+03 | -3.141 (0.130) |
| deg off, bias frozen | 4.011 | -3.282 | 4.7 | 24 | 0.0776 | 1.3e+03 | 1.65e+03 | -3.282 (0.133) |
| deg off, relin 0.01 | 4.518 | -4.200 | 6.0 | 24 | 0.0635 | 1.15e+03 | 1.35e+03 | -4.200 (0.133) |
| deg off, relin 1.0 | 4.232 | -3.333 | 4.7 | 23 | 0.0732 | 1.18e+03 | 1.31e+03 | -3.673 (0.135) |
| deg off, KF every 20 scans | 3.861 | -2.761 | 3.9 | 9 | 0.0888 | 4.07e+03 | 4.19e+03 | -3.923 (0.133) |
| deg off, KF every 5 scans | 6.704 | -8.098 | 11.5 | 36 | 0.0810 | 1.86e+03 | 2.04e+03 | -8.545 (0.131) |
| deg off, oracle measurement | 0.317 | +0.219 | 0.3 | 24 | 0.1629 | 1.22e-28 | 0 | -0.006 (0.140) |
| deg off, tilt prior 0.02 deg | 4.183 | -4.292 | 6.1 | 23 | 0.0693 | 884 | 1.03e+03 | -4.585 (0.122) |
| deg off, tight kf0 priors | 4.483 | -3.982 | 5.7 | 24 | 0.0872 | 1.41e+03 | 1.67e+03 | -3.982 (0.120) |
| deg on, baseline | 0.023 | -0.039 | 0.1 | 23 | 0.0001 | 0.305 | 11.9 | -0.036 (0.136) |
| deg on, no backend feedback | 0.245 | +0.578 | 0.8 | 23 | 0.0001 | 8.18 | 12.4 | +0.557 (0.135) |
| deg on, bias frozen | 0.167 | +0.406 | 0.6 | 23 | 0.0001 | 4.17 | 8.15 | +0.387 (0.134) |
| deg on, relin 0.01 | 0.023 | -0.039 | 0.1 | 23 | 0.0001 | 0.305 | 11.9 | -0.036 (0.136) |
| deg on, relin 1.0 | 0.023 | -0.039 | 0.1 | 23 | 0.0001 | 0.305 | 11.9 | -0.036 (0.136) |
| deg on, KF every 20 scans | 0.280 | +0.722 | 1.0 | 9 | 0.0001 | 36.4 | 41 | +0.525 (0.133) |
| deg on, KF every 5 scans | 20.368 | -48.985 | 69.6 | 36 | 0.0042 | 4.89e+03 | 5.13e+03 | -46.490 (0.168) |
| deg on, oracle measurement | 0.003 | -0.003 | 0.0 | 23 | 0.0001 | 1.84e-28 | 0 | -0.003 (0.136) |
| deg on, tilt prior 0.02 deg | 0.040 | +0.106 | 0.2 | 23 | 0.0001 | 0.344 | 4.18 | +0.098 (0.121) |
| deg on, tight kf0 priors | 0.153 | +0.368 | 0.5 | 23 | 0.0001 | 3.02 | 6.69 | +0.350 (0.120) |
| deg on, x free, baseline | 99.338 | +224.329 | 318.5 | 115 | 0.0210 | 1.47e+03 | 2.18e+03 | +224.329 (0.659) |
| deg on, x free, no backend feedback | 0.245 | +0.578 | 0.8 | 23 | 0.0001 | 8.18e-07 | 4.24 | +0.681 (2.166) |
| deg on, x free, bias frozen | 9.816 | +15.966 | 22.7 | 27 | 0.0087 | 0.000251 | 772 | +15.293 (0.974) |
| deg on, x free, relin 0.01 | 38.775 | +81.655 | 115.9 | 58 | 0.0154 | 0.000257 | 608 | +81.655 (1.201) |
| deg on, x free, relin 1.0 | 28.086 | +54.370 | 77.2 | 45 | 0.0654 | 1.32e+03 | 2.22e+03 | +54.338 (0.631) |
| deg on, x free, KF every 20 scans | 5.199 | +9.956 | 14.1 | 9 | 0.0002 | 0.000387 | 81.3 | +7.889 (0.737) |
| deg on, x free, KF every 5 scans | 12.453 | -27.263 | 38.7 | 36 | 0.0062 | 21 | 90 | -26.040 (1.004) |
| deg on, x free, oracle measurement | 0.438 | +0.505 | 0.7 | 23 | 0.0001 | 1.55e-35 | 0 | +0.481 (1.366) |
| deg on, x free, tilt prior 0.02 deg | 0.377 | -0.803 | 1.1 | 23 | 0.0001 | 2.03e-07 | 4.32 | -0.796 (1.684) |
| deg on, x free, tight kf0 priors | 0.176 | +0.408 | 0.6 | 23 | 0.0001 | 3.23e-07 | 3.57 | +0.391 (0.800) |
| deg on, x removed, baseline | 91.393 | +316.046 | 448.8 | 53 | 0.0239 | 4.38e-06 | 568 | +316.046 (17.544) |
| deg on, x removed, no backend feedback | 0.403 | -1.498 | 2.1 | 27 | 0.0262 | 6.45e-08 | 1.99e+03 | +5772.337 (14.051) |
| deg on, x removed, bias frozen | 28.871 | +83.306 | 118.3 | 38 | 0.0190 | 1.2e-06 | 470 | +83.306 (4.284) |
| deg on, x removed, relin 0.01 | 21.374 | +86.045 | 122.2 | 33 | 0.0151 | 1.05e-06 | 346 | +85.082 (13.349) |
| deg on, x removed, relin 1.0 | 106.826 | +1110.863 | 1577.3 | 41 | 0.0067 | 3.16e-06 | 193 | +1110.863 (1.545) |
| deg on, x removed, KF every 20 scans | 5.223 | +16.072 | 22.8 | 9 | 0.0008 | 3.13e-07 | 71.7 | +12.806 (8.913) |
| deg on, x removed, KF every 5 scans | 112.074 | +385.304 | 547.1 | 36 | 0.0295 | 5.35e-05 | 293 | +368.044 (18.896) |
| deg on, x removed, oracle measurement | 2.848 | -3.677 | 5.2 | 23 | 0.0002 | 3.16e-38 | 0 | -3.521 (18.088) |
| deg on, x removed, tilt prior 0.02 deg | 0.130 | -0.157 | 0.2 | 23 | 0.0001 | 2.06e-10 | 3.67 | -0.162 (1.995) |
| deg on, x removed, tight kf0 priors | 0.123 | +0.267 | 0.4 | 23 | 0.0001 | 2.7e-10 | 3.29 | +0.254 (0.847) |
| deg off, collinearity 0.1, baseline | 4.708 | -4.487 | 6.4 | 23 | 0.0912 | 829 | 999 | -4.771 (0.145) |
| deg off, collinearity 0.1, no backend feedback | 1.380 | +0.262 | 0.4 | 23 | 0.0967 | 609 | 1.35e+03 | +0.455 (0.127) |
| deg on, x free, thr 1e4, baseline | 7.734 | -18.421 | 26.2 | 24 | 0.0004 | 0.00076 | 74.4 | -17.687 (1.196) |
| deg on, x free, thr 1e4, no backend feedback | 3.215 | +9.976 | 14.2 | 25 | 0.0033 | 0.000355 | 240 | +27.455 (0.838) |

### Straight

With degeneracy off the straight corridor never reaches 1 m of estimated travel: the matcher
pins x to keyframe 0 (section 6), so one keyframe in 20 s and no backend number.

| case | along-x RMS (m) | end x err (m) | end / sigma_DR | keyframes | matcher x corr RMS (m) | LiDAR factor x NEES (exp. 1) | LiDAR factor 6-DoF NEES (exp. 6) | backend last KF x err (sigma) (m) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| deg off, baseline | 4.346 | -7.940 | 11.3 | 1 | 0.0482 | 0 | 0 | +0.000 (0.000) |
| deg off, no backend feedback | 4.346 | -7.940 | 11.3 | 1 | 0.0482 | 0 | 0 | +0.000 (0.000) |
| deg off, bias frozen | 4.346 | -7.940 | 11.3 | 1 | 0.0482 | 0 | 0 | +0.000 (0.000) |
| deg off, relin 0.01 | 4.346 | -7.940 | 11.3 | 1 | 0.0482 | 0 | 0 | +0.000 (0.000) |
| deg off, relin 1.0 | 4.346 | -7.940 | 11.3 | 1 | 0.0482 | 0 | 0 | +0.000 (0.000) |
| deg off, KF every 20 scans | 4.333 | -7.952 | 11.3 | 9 | 0.0208 | 3.88e+05 | 4.2e+05 | -6.997 (0.100) |
| deg off, KF every 5 scans | 4.377 | -8.075 | 11.5 | 36 | 0.0097 | 1.66e+04 | 2.85e+04 | -7.874 (0.101) |
| deg off, oracle measurement | 4.346 | -7.940 | 11.3 | 1 | 0.0482 | 0 | 0 | +0.000 (0.000) |
| deg off, tilt prior 0.02 deg | 4.346 | -7.940 | 11.3 | 1 | 0.0482 | 0 | 0 | +0.000 (0.000) |
| deg off, tight kf0 priors | 4.346 | -7.940 | 11.3 | 1 | 0.0482 | 0 | 0 | +0.000 (0.000) |
| deg on, baseline | 0.051 | -0.064 | 0.1 | 8 | 0.0001 | 0.345 | 1.42 | -0.080 (0.118) |
| deg on, no backend feedback | 0.025 | +0.097 | 0.1 | 8 | 0.0001 | 0.726 | 2.16 | +0.051 (0.118) |
| deg on, bias frozen | 0.061 | -0.069 | 0.1 | 8 | 0.0001 | 0.533 | 1.84 | -0.091 (0.118) |
| deg on, relin 0.01 | 0.048 | -0.051 | 0.1 | 8 | 0.0001 | 0.323 | 1.51 | -0.072 (0.118) |
| deg on, relin 1.0 | 0.051 | -0.064 | 0.1 | 8 | 0.0001 | 0.345 | 1.42 | -0.080 (0.118) |
| deg on, KF every 20 scans | 0.067 | -0.039 | 0.1 | 9 | 0.0001 | 0.515 | 2.14 | -0.081 (0.122) |
| deg on, KF every 5 scans | 11.614 | -26.915 | 38.2 | 36 | 0.0023 | 1.8e+03 | 3.41e+03 | -25.622 (0.145) |
| deg on, oracle measurement | 0.005 | -0.002 | 0.0 | 8 | 0.0001 | 1.07e-28 | 0 | +0.001 (0.117) |
| deg on, tilt prior 0.02 deg | 0.054 | -0.055 | 0.1 | 8 | 0.0001 | 0.422 | 2.05 | -0.077 (0.118) |
| deg on, tight kf0 priors | 0.052 | -0.048 | 0.1 | 8 | 0.0001 | 0.377 | 1.74 | -0.074 (0.117) |
| deg on, x free, baseline | 1.171 | +3.556 | 5.0 | 10 | 0.0002 | 1.57e-05 | 45.5 | +3.556 (0.505) |
| deg on, x free, no backend feedback | 0.025 | +0.097 | 0.1 | 8 | 0.0001 | 7.26e-08 | 1.43 | -0.065 (1.387) |
| deg on, x free, bias frozen | 1.192 | -2.813 | 4.0 | 6 | 0.0001 | 2.52e-05 | 23.3 | -2.193 (0.908) |
| deg on, x free, relin 0.01 | 0.814 | +0.982 | 1.4 | 9 | 0.0003 | 1.03e-05 | 23.2 | +0.955 (0.521) |
| deg on, x free, relin 1.0 | 0.827 | +2.155 | 3.1 | 9 | 0.0002 | 7.5e-06 | 27.1 | +1.950 (0.624) |
| deg on, x free, KF every 20 scans | 5.513 | -12.507 | 17.8 | 9 | 0.0001 | 0.0013 | 2.89e+03 | -9.748 (0.413) |
| deg on, x free, KF every 5 scans | 4.646 | -8.264 | 11.7 | 36 | 0.0033 | 0.000117 | 3.22e+03 | -8.093 (0.116) |
| deg on, x free, oracle measurement | 0.237 | -0.540 | 0.8 | 8 | 0.0001 | 8.84e-36 | 0 | -0.496 (1.068) |
| deg on, x free, tilt prior 0.02 deg | 0.333 | +1.174 | 1.7 | 8 | 0.0002 | 2.45e-06 | 24.9 | +0.988 (0.492) |
| deg on, x free, tight kf0 priors | 0.296 | -0.373 | 0.5 | 8 | 0.0001 | 1.65e-06 | 7.95 | -0.370 (0.530) |
| deg on, x removed, baseline | 0.812 | -7.504 | 10.7 | 9 | 0.0001 | 1.51e-08 | 3.71 | -7.504 (25.491) |
| deg on, x removed, no backend feedback | 0.025 | +0.097 | 0.1 | 8 | 0.0001 | 3.13e-10 | 1.4 | +11.373 (20.896) |
| deg on, x removed, bias frozen | 0.589 | -1.956 | 2.8 | 8 | 0.0001 | 3.37e-09 | 1.26 | -1.603 (22.094) |
| deg on, x removed, relin 0.01 | 0.766 | +3.853 | 5.5 | 8 | 0.0001 | 5.63e-08 | 4.2 | +3.853 (29.213) |
| deg on, x removed, relin 1.0 | 8.332 | -10.117 | 14.4 | 11 | 0.0003 | 6.18e-07 | 80.6 | -9.798 (19.728) |
| deg on, x removed, KF every 20 scans | 1.002 | -3.045 | 4.3 | 9 | 0.0001 | 1.93e-08 | 1.42 | -2.405 (25.029) |
| deg on, x removed, KF every 5 scans | 65.625 | +252.058 | 357.9 | 36 | 0.0066 | 1.2e-05 | 249 | +240.762 (23.153) |
| deg on, x removed, oracle measurement | 2.777 | +5.295 | 7.5 | 10 | 0.0002 | 7.29e-38 | 0 | +4.842 (19.749) |
| deg on, x removed, tilt prior 0.02 deg | 0.134 | +0.141 | 0.2 | 8 | 0.0001 | 1.69e-09 | 1.13 | +0.126 (14.050) |
| deg on, x removed, tight kf0 priors | 0.088 | -0.171 | 0.2 | 8 | 0.0001 | 1.95e-09 | 2.12 | -0.165 (1.324) |
| deg off, collinearity 0.1, baseline | 4.345 | -7.946 | 11.3 | 1 | 0.0429 | 0 | 0 | +0.000 (0.000) |
| deg off, collinearity 0.1, no backend feedback | 4.345 | -7.946 | 11.3 | 1 | 0.0429 | 0 | 0 | +0.000 (0.000) |
| deg on, x free, thr 1e4, baseline | 4.774 | -7.130 | 10.1 | 7 | 0.0002 | 0.00223 | 3.2 | -6.781 (3.522) |
| deg on, x free, thr 1e4, no backend feedback | 1.078 | -3.139 | 4.5 | 6 | 0.0001 | 9.52e-05 | 2.09 | -6.811 (5.720) |

### Backend tilt and bias in the failing runs (circle)

`theta_y` is the backend attitude error about world y (the horizontal axis normal to the
corridor), the tilt that leaks gravity along world x.

| case | t (s) | x err (m) | v_x err (m/s) | ba_w x / y (m/s^2) | theta_x / theta_y (mrad) |
| --- | --- | --- | --- | --- | --- |
| deg on, x free, baseline | 4.7 | -0.001 | +0.000 | +0.0002 / -0.0010 | +0.32 / +0.22 |
| deg on, x free, baseline | 8.0 | +9.44 | +3.17 | -0.0233 / -0.0112 | -0.02 / +53.0 |
| deg on, x free, baseline | 11.0 | +45.8 | +10.5 | +0.0079 / +0.0114 | -2.26 / +125.0 |
| deg on, x free, baseline | 19.9 | +224.3 | +25.5 | +0.0122 / +0.0047 | -0.01 / +149.8 |
| deg on, x removed, baseline | 11.8 | -4.36 | -0.90 | +0.0230 / +0.0029 | -0.76 / -8.5 |
| deg on, x removed, baseline | 15.0 | +42.1 | +6.46 | +0.0092 / +0.0164 | -0.50 / +50.5 |
| deg on, x removed, baseline | 19.9 | +316.0 | +35.5 | +0.0484 / -0.0073 | +1.46 / +212.4 |
| deg on, x free, tight kf0 priors | 19.5 | +0.391 | +0.043 | -0.0006 / +0.0014 | +0.05 / +0.40 |
| deg on, x removed, tight kf0 priors | 19.5 | +0.254 | +0.032 | -0.0008 / +0.0013 | +0.09 / +0.32 |

`g sin(0.150 rad)` = 1.47 m/s^2 along x: the 224 m are gravity leaking through a 150 mrad
tilt that nothing pins, not accelerometer bias (|ba_w| stays below 0.05 m/s^2).

### Exploratory controls (temporary patches, not committed)

Two backend changes were tried on the "x removed" rows with a temporary patch, to separate
the factor and the optimiser from the priors:

| change | circle x removed: baseline / no feedback (end x err, backend last KF) | straight x removed: baseline / no feedback |
| --- | --- | --- |
| none (shipped `BetweenFactor<Pose3>`, 1 iSAM2 update) | +316 m / backend +5772 m | -7.5 m / backend +11.4 m |
| LiDAR factor with translation and rotation residuals decoupled, exact Jacobians | +290 m / backend -8.7 m (sigma 24.5) | -3.5 m / +4.4 m |
| 5 extra `ISAM2::update()` per keyframe | +117 m / backend -10.7 m (sigma 25.5) | -1.2 m / +4.9 m |

Both repair the backend-only divergence of the "no feedback" row (there the measurement's x,
taken from the front end, differs from the backend's by kilometres, and the SE(3) residual
`Log(Z^-1 T_i^-1 T_j)` of `BetweenFactor` leaks that difference into the observed
components through `V(phi)^-1`, with a Jacobian that omits `Jr^-1`, `backend_factors.md`
section 5), but neither repairs the feedback rows: they are amplifiers, not the cause.

## 2. LiDAR factor along x removed vs present

Removing the information along x does not make the drift track dead reckoning; it makes it
far worse. Circle: present (deg on, the factor's x is the IMU prediction with a millimetre
sigma, x NEES 0.3) -0.04 m (0.1 sigma_DR); x free +224 m (319 sigma_DR, 115 keyframes);
x removed exactly +316 m (449 sigma_DR). Straight: -0.06 m, +3.6 m, -7.5 m. The backend
claims 0.66 m (x free) and 17.5 m (x removed) for errors of 224 m and 316 m.

## 3. Front-end feedback

Without backend feedback the x-free run is consistent: circle +0.58 m published, backend
+0.68 m with sigma 2.2 m; straight +0.10 m. With the shipped "deg on" feedback helps (-0.04
vs +0.58 m: the backend's smoothed velocity reaches the prediction). Feedback is therefore
not wrong in itself; it is the path by which a backend excursion along the free direction
reaches the map and the next measurements (section 8). Feedback alone does not explain
deg off (3.4 m without it) nor fixed-period keyframes (KF every 5 scans diverges without
feedback too, in the study run of the exploration: 87 m circle deg on).

## 4. Accelerometer bias

Freezing the bias at its true value improves x free (224 -> 16 m) and x removed (316 -> 83
m) but leaves them 20 to 120 sigma_DR off, and the backend bias in the failing runs stays
below 0.05 m/s^2 in w (table above): the bias does not absorb the x velocity. Candidate 4 as
stated is not the mechanism; the bias prior is a secondary path of the same null space
(section 8).

## 5. Relinearisation

`relinearize_threshold` 0.01 / 1.0 changes x free from 224 m to 82 / 54 m and x removed to
86 / 1111 m, deg on not at all (-0.039 m). Not the cause; with a nearly free direction the
single Gauss-Newton step per keyframe lands at different points of the same null space.

## 6. Keyframe decision and spurious x information

The keyframe count rises with the drift (23 -> 115 on x free) because the 1 m rule fires on
the drifting x; fixed-period keyframes every 20 scans (2 s, 9 keyframes) still drift 10 m
(x free) and 16 m (x removed), so the rule is an amplifier, not the cause. Keyframes every 5
scans make every setting diverge (deg on: 49 m circle, 27 m straight): a denser map gives
more spurious information along x (median Hessian information along x 30 vs 6 1/m^2, full
table), so flagged directions stop being flagged on some scans and those factors report
pinned x with millimetre sigmas (factor x NEES 4890).

Spurious x information is also the whole story of **deg off** (main's shipped setting): the
matcher corrects x by 6.8 cm RMS per scan where nothing observes x, and each LiDAR factor
claims 1.4 cm along x for a 0.46 m error (x NEES 1070). The 5-NN plane fits on the floor
and ceiling often take their neighbours along one ring arc, whose fitted normal has a
horizontal component; Gauss-Newton then locks the scan's ring pattern onto the map's, which
pins x to where the keyframes were inserted (the straight corridor never travels 1 m:
one keyframe, -7.9 m). The oracle measurement removes it (0.22 m); the collinearity gate
(`scan_match.min_mid_eig_ratio` 0.1) does not (x NEES 829); a flag threshold of 1e4 (row
"thr 1e4") flags rotations too and loses the attitude (lateral error 0.72 m). Deg off is a
degeneracy-detection problem (PR 4b enables degeneracy by default), outside this PR.

## 7. Oracle measurement

With the true relative pose as measurement and the same covariance, every setting is within
DR: deg off +0.22 m, deg on -0.003 m, x free +0.51 m (circle). With x removed the oracle run
is consistent but wide: -3.7 m with backend sigma 18 m (circle), +5.3 m with sigma 20 m
(straight): even perfect LiDAR relative poses leave the backend's x uncertain by tens of
metres when nothing else pins the tilt.

## 8. Mechanism

The candidates of the brief, with the numbers above:

| candidate | verdict |
| --- | --- |
| 1. backend state fed into the front-end prediction, reported by the matcher | amplifier when x is free (section 3); harmless when x is tight (deg on) |
| 2. LiDAR between factor from a scan-to-map covariance | with deg off the matcher reports spurious x confidently (x NEES 1070), the deg-off defect; with deg on it double-counts the prediction consistently (NEES 0.3) |
| 3. iSAM2 relinearisation along a nearly free direction | changes where the estimate lands, not whether it drifts (section 5) |
| 4. accelerometer-bias prior 0.1 m/s^2 | secondary path (section 4) |
| 5. keyframe rule reacting to the drift | amplifier (section 6) |

None of them is the root. The root is the **attitude anchor**: with world x unobserved, a
tilt theta about the horizontal axis normal to x is indistinguishable, in the IMU factors,
from an acceleration `g theta` along x (and, in the static window, from a horizontal
accelerometer bias); the LiDAR observes tilt only relative to its own map, which is built
from the estimated keyframe poses. The only absolute tilt information in the graph is the
keyframe-0 prior, `backend.prior_rot_sigma_deg` = 1 deg = 17.5 mrad per axis, 42 times
looser than what the static window measured (0.42 mrad). That prior alone allows
`0.5 g theta T^2` = 27 m at 17.9 s (the backend's own 17.5 m sigma in the x-removed rows).
With feedback, a tilt excursion of the backend is written into the map (keyframe clouds
inserted at the optimised pose, ADR-016 rebuilds), the next matches follow the tilted map
(factor 6-DoF NEES 2180 on x free, mostly rotation), and the LiDAR chain stops resisting:
theta reaches 150 to 212 mrad and x 224 to 316 m.

Evidence that this is the cause: tightening only the keyframe-0 rotation prior to 0.02 deg
brings x free to -0.80 m (circle) and +1.17 m (straight), x removed to -0.16 / +0.14 m; adding
the bias (0.004 m/s^2) and velocity (0.01 m/s) priors gives +0.41 / -0.37 m (x free) and
+0.27 / -0.17 m (x removed), all within 1.7 sigma_DR, with the tilt below 0.5 mrad and the
backend sigma consistent (0.53 to 1.3 m). Deg on and deg off are unchanged within noise
(deg off stays at its spurious-information error).

A tight tilt prior is the right size but the wrong form: the static window measures the
specific force `f = ba - R^T g_w`, so a real horizontal accelerometer bias `b` makes the
initial tilt wrong by `b / g`, and a tight tilt prior would pin that wrong tilt (the sim
noise profile draws biases of 0.3 m/s^2 sigma). The fix (section 9) therefore adds the
static window's measurement itself to the graph.

## 9. Fix, round 1: the static-window gravity factor (full residual, evaluated, then off)

*Historical (round 1 of this PR).* The rows of this section were measured with the full 3x1
residual before the round-2 changes. They are reproduced at the head of the branch through the
study seam (`ImuStaticGravityRound2Seeds`, variant "full residual"; section 10e); the factor
that ships is the one of section 10.

The static window measures the specific force `f = ba - R_w_b^T g_w` with the covariance of
its mean (4.1e-3 m/s^2 per axis here). The keyframe-0 priors replaced that measurement by an
independent 1 deg tilt prior and a 0.1 m/s^2 bias prior, which is where the 17.5 mrad of
freedom of section 8 comes from. `backend.static_gravity_factor` adds the measurement itself
as a factor on keyframe 0's pose and bias (`backend_factors.md` section 6c): it ties the
tilt and the horizontal bias together as the window does, so it stays correct with a real
bias, where a tight tilt prior would pin the wrong tilt (`b / g`). The pipeline passes the
initializer's window statistics (`imu::InitState::acc_mean_b`, `acc_mean_cov`) when keyframe
0 is the initialisation scan. The keyframe-0 priors are unchanged.

Seed 42, the study tables of sections 2 to 7 with the factor on (the other rows are
unchanged; full table in `results/<run-id>/imu_study.md`):

| scene | case | end x err (m) | end / sigma_DR | keyframes | LiDAR factor 6-DoF NEES | backend last KF x err (sigma) (m) |
| --- | --- | --- | --- | --- | --- | --- |
| circle | deg off | -2.58 (main -4.36) | 3.7 | 24 | 1620 | -2.58 (0.135) |
| circle | deg on | +0.12 (main -0.04) | 0.2 | 23 | 8.1 | +0.11 (0.136) |
| circle | deg on, x free | +0.90 (main +224) | 1.3 | 23 | 44.5 | +0.78 (1.20) |
| circle | deg on, x removed | +1.86 (main +316) | 2.6 | 23 | 43.1 | +1.70 (2.03) |
| straight | deg on | -0.29 (main -0.06) | 0.4 | 8 | 4.7 | -0.27 (0.118) |
| straight | deg on, x free | -1.94 (main +3.56) | 2.8 | 7 | 42.4 | -1.67 (0.665) |
| straight | deg on, x removed | -0.44 (main -7.50) | 0.6 | 8 | 1.3 | -0.39 (2.10) |
| circle, bias (0.05, 0.03, 0) m/s^2 | deg on | -4.76 (main -4.97, tilt prior -6.59) | 6.8 | 23 | 382 | -4.49 (0.136) |
| circle, bias (0.05, 0.03, 0) m/s^2 | deg on, x free | -5.21 (main -19.7, tilt prior -9.49) | 7.4 | 23 | 205 | -5.09 (1.09) |

Keyframes every 5 scans stay bad with the factor (deg on 21.9 m circle): that failure is
the spurious information of section 6, not the tilt.

Five seeds (42 to 46, IMU and GNSS noise; `PipelineComponent.ImuOnlyObservabilityFixEvaluation`,
`results/<run-id>/imu_fix_eval.md`), against the prior variants of section 8 and a zero-velocity
prior (keyframe 0 ends a window the initializer accepted as static):

| scene | metric | variant | median (m) | max (m) | per seed (m) |
| --- | --- | --- | --- | --- | --- |
| 20 s circle corridor, LiDAR + IMU, x free | end x err | main (no factor) | 84.816 | 224.329 | +224.329 -13.437 -84.816 +150.183 -28.051 |
| 20 s circle corridor, LiDAR + IMU, x free | end x err | static gravity factor | 0.903 | 7.410 | +0.903 -0.107 -0.106 +7.410 -2.795 |
| 20 s circle corridor, LiDAR + IMU, x free | end x err | tight kf0 priors | 0.408 | 1.414 | +0.408 +0.210 +0.447 -0.089 -1.414 |
| 20 s circle corridor, LiDAR + IMU, x free | end x err | factor + tight priors | 0.296 | 1.160 | +0.473 +0.215 +0.296 +0.202 -1.160 |
| 20 s circle corridor, LiDAR + IMU, x free | end x err | factor + velocity prior 0.01 m/s | 0.498 | 5.060 | -0.498 -0.170 -2.172 +5.060 +0.381 |
| 20 s straight corridor, LiDAR + IMU, x free | end x err | main (no factor) | 4.284 | 14.657 | +3.556 -6.904 +14.657 -4.284 +1.582 |
| 20 s straight corridor, LiDAR + IMU, x free | end x err | static gravity factor | 1.939 | 4.552 | -1.939 +1.171 +1.812 +4.552 +3.109 |
| 20 s straight corridor, LiDAR + IMU, x free | end x err | tight kf0 priors | 0.453 | 1.209 | -0.373 -1.209 +0.187 -0.453 -0.855 |
| 20 s straight corridor, LiDAR + IMU, x free | end x err | factor + tight priors | 0.465 | 1.009 | -0.033 -0.465 -0.372 +0.482 -1.009 |
| 20 s straight corridor, LiDAR + IMU, x free | end x err | factor + velocity prior 0.01 m/s | 1.047 | 1.344 | -1.344 -0.469 +1.128 +1.047 -0.964 |
| 60 s corridor (CorridorDriftCorrectedByGnss), LiDAR + IMU | ATE | main (no factor) | 21.560 | 51.565 | +15.357 +21.560 +19.421 +51.565 +24.631 |
| 60 s corridor (CorridorDriftCorrectedByGnss), LiDAR + IMU | ATE | static gravity factor | 28.449 | 56.102 | +56.102 +6.448 +17.401 +28.449 +39.406 |
| 60 s corridor (CorridorDriftCorrectedByGnss), LiDAR + IMU | ATE | tight kf0 priors | 23.938 | 60.153 | +19.895 +31.970 +23.283 +23.938 +60.153 |
| 60 s corridor (CorridorDriftCorrectedByGnss), LiDAR + IMU | ATE | factor + tight priors | 16.679 | 29.256 | +4.485 +14.118 +16.679 +29.256 +18.892 |
| 60 s corridor (CorridorDriftCorrectedByGnss), LiDAR + IMU | ATE | factor + velocity prior 0.01 m/s | 22.671 | 45.811 | +22.671 +10.455 +40.803 +45.811 +11.230 |
| 60 s corridor (CorridorDriftCorrectedByGnss), + GNSS 0.5 m | ATE | main (no factor) | 0.636 | 7.574 | +1.820 +7.574 +0.636 +0.627 +0.587 |
| 60 s corridor (CorridorDriftCorrectedByGnss), + GNSS 0.5 m | ATE | static gravity factor | 4.153 | 5.225 | +5.225 +0.745 +3.041 +4.153 +4.656 |
| 60 s corridor (CorridorDriftCorrectedByGnss), + GNSS 0.5 m | ATE | tight kf0 priors | 0.587 | 4.668 | +0.546 +0.402 +0.587 +4.668 +0.825 |
| 60 s corridor (CorridorDriftCorrectedByGnss), + GNSS 0.5 m | ATE | factor + tight priors | 0.798 | 4.061 | +0.798 +0.561 +2.175 +4.061 +0.564 |
| 60 s corridor (CorridorDriftCorrectedByGnss), + GNSS 0.5 m | ATE | factor + velocity prior 0.01 m/s | 1.951 | 3.034 | +0.761 +0.416 +1.951 +2.583 +3.034 |
| box world, LiDAR + IMU (shipped degeneracy off) | ATE | main (no factor) | 0.009 | 0.011 | +0.009 +0.009 +0.009 +0.011 +0.010 |
| box world, LiDAR + IMU (shipped degeneracy off) | ATE | static gravity factor | 0.009 | 0.011 | +0.009 +0.009 +0.009 +0.011 +0.011 |
| box world, LiDAR + IMU (shipped degeneracy off) | ATE | tight kf0 priors | 0.009 | 0.011 | +0.009 +0.009 +0.009 +0.011 +0.010 |
| box world, LiDAR + IMU (shipped degeneracy off) | ATE | factor + tight priors | 0.009 | 0.011 | +0.009 +0.009 +0.009 +0.010 +0.011 |
| box world, LiDAR + IMU (shipped degeneracy off) | ATE | factor + velocity prior 0.01 m/s | 0.010 | 0.011 | +0.009 +0.010 +0.009 +0.011 +0.011 |
| 20 s circle corridor, x free, acc bias (0.05, 0.03, 0) m/s^2 | end x err | main (no factor) | 10.359 | 31.015 | -19.738 +5.649 +10.359 -31.015 +0.810 |
| 20 s circle corridor, x free, acc bias (0.05, 0.03, 0) m/s^2 | end x err | static gravity factor | 5.213 | 9.905 | -5.213 -4.042 -9.905 +0.718 -5.778 |
| 20 s circle corridor, x free, acc bias (0.05, 0.03, 0) m/s^2 | end x err | tight kf0 priors | 7.513 | 8.263 | -8.263 -7.693 -7.513 -6.533 -6.184 |
| 20 s circle corridor, x free, acc bias (0.05, 0.03, 0) m/s^2 | end x err | factor + tight priors | 5.562 | 7.226 | -5.210 -3.908 -5.562 -6.120 -7.226 |
| 20 s circle corridor, x free, acc bias (0.05, 0.03, 0) m/s^2 | end x err | factor + velocity prior 0.01 m/s | 3.811 | 7.737 | -7.737 -1.169 +4.384 +2.322 -3.811 |

Round-1 decision (superseded by section 10, which ships the direction-only form on): the
full-residual factor at the window sigma stayed **off** (the previous behaviour). It does
what the diagnosis asks on the scenes it was built for: the IMU-only corridor with x
unobserved goes from a median of 85 m to 0.90 m (circle) and 4.3 m to 1.9 m (straight), and
with a 0.06 m/s^2 bias from 10.4 m to 5.2 m, where the tight priors (7.5 m) pin the wrong
tilt. But it is not better everywhere, so the flag rule (CLAUDE.md, DoD 3) keeps it off:

- the 60 s GNSS corridor gets worse in median, 0.64 -> 4.15 m (seed 42: 1.82 -> 5.23 m, over
  the 2.18 m guard of `CorridorDriftCorrectedByGnss`), although its worst seed improves
  (7.57 -> 5.23 m);
- the LiDAR factor NEES on the box world rises from 7.8 to 10.8 (bound 12, test
  `LidarFactorCovarianceConsistent`); box-world ATE is unchanged (9.0 / 9.4 mm);
- across seeds the circle still has one 7.4 m run (seed 45) and the straight corridor 4.6 m
  (seed 45): the factor removes the initial-tilt freedom, not the slower tilt random walk of
  the LiDAR chain, which dominates the 60 s corridor (LiDAR only 15 to 56 m in every
  variant, tilt errors of 1 to 20 mrad over the run).

With the zero-velocity prior (`backend.prior_vel_sigma_mps` 0.01) the factor is better on
the 20 s corridors (0.50 / 1.05 m) and on the biased scene (3.8 m), and the 60 s GNSS
corridor improves in mean and maximum (2.25 -> 1.75 m, 7.57 -> 3.03 m) but not in median
(0.64 -> 1.95 m); the box-world NEES reaches 11.95 and `CorridorOutageRealignsAfterRejects`
loses its factors after the outage (0 instead of 11). Not shipped either.

What remains open (follow-up): the tilt about the horizontal axis normal to an unobserved
direction is anchored only by keyframe 0 and the LiDAR chain; a consistent fix needs the
GNSS / LiDAR tilt interplay of the 60 s corridor understood (E's tight tilt prior is centred
on the aligner's gravity-aligned guess) before the factor can be turned on. The regression
test `CorridorImuOnlyAlongAxisWithinDeadReckoning` enables the factor by overlay and bounds
the end error by `3 sigma_DR` (2.11 m at 17.9 s; measured 0.90 m; main: 224 m).

### Integration and bench, round 1 (flag off; round 2 in section 10f)

E-01..E-03 data of the integration harness (synthetic box circle, R = 1.67 m, 60 s, seed 42),
`mss_bench` with `config/sim/warehouse.yaml`, ATE by `mss_eval.ate`; main = `94b9f16`:

| test | main ATE (m) | branch ATE (m) | branch with `static_gravity_factor` (m) |
| --- | --- | --- | --- |
| E-01 | 0.002199 | 0.002199 (`est.tum` byte-identical to main) | 0.002116 |
| E-02 | byte-identical rerun | byte-identical rerun | - |
| E-03 | 0.002105 | 0.002105 | 0.002097 |

E-01 bench, median of 5 (desktop, WSL, interleaved runs), main / branch: scan p50 4.373 /
4.295 ms, p95 5.957 / 5.928 ms, `replay_wall_s` 2.606 / 2.546 s, CPU 100 / 99 %. The per-scan
path gains one null-pointer test (the study observer) and one branch (the study keyframe
rule); the factor is added once, at keyframe 0.

## 10. Round 2: why the factor seemed to degrade the GNSS corridor, and the shipped form

Driver: `PipelineComponent.ImuStaticGravityRound2Study` (5 seeds, about 5 min) and
`ImuStaticGravityRound2Seeds` (15 seeds, about 20 min per run of all variants) in
`tests/component/test_imu_only_study.cpp` (`MSS_IMU_STUDY=1`), written to
`results/<run-id>/imu_round2.md` and `imu_round2_seeds.md`. Every row below was regenerated at
the head of the branch.

- Rows labelled "full residual" use the study seam `detail::set_static_gravity_full_residual`
  (`core/backend/src/study_seams.hpp`), which restores the round-1 3x1 residual. They reproduce
  round 1: box NEES 9.60 and the per-seed GNSS-corridor values, within a few percent where the
  consistency check of section 12 now runs. The full residual's chi-square has 3 DoF, so the
  check is more permissive there.
- Rows labelled "direction only" or "shipped form" are the product factor.
- The noise variants use the hook `CoreTestHooks::static_gravity_cov` with
  `backend.static_gravity_sigma_scale: 1`.
- `E` is read through `Graph::gnss_frame_estimate()`.

Scenes:
- the 60 s corridor of `CorridorDriftCorrectedByGnss`: straight, 10 m range, degeneracy relax
  1e8, GNSS 0.5 m;
- the box world: LiDAR + IMU, with the NEES of `LidarFactorCovarianceConsistent`.

### 10a, b, d. Variants over seeds 42..46

| variant | GNSS corridor ATE median (max) (m) | per seed (m) | box NEES median (exp. 6) | per seed | end E yaw err median (mrad) | end E pos err median (m) | end kf tilt err median (mrad) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| main (factor off) | 0.64 (7.57) | 1.82 7.57 0.64 0.63 0.59 | 8.55 | 7.8 6.7 12.6 8.5 8.6 | 5.28 | 0.45 | 6.06 |
| full residual, window sigma | 4.15 (5.03) | 5.03 0.74 3.04 4.15 4.66 | 9.60 | 10.8 7.1 13.8 7.4 9.6 | 2.68 | 1.59 | 8.44 |
| full residual, sigma / 10 | 2.28 (22.30) | 22.30 0.41 2.28 0.86 3.24 | 10.17 | 11.9 8.3 15.2 6.6 10.2 | 5.95 | 0.96 | 15.20 |
| full residual, sigma x 3 | 2.61 (3.98) | 3.98 0.50 3.33 0.30 2.61 | 7.81 | 7.8 5.8 11.3 8.1 7.6 | 4.50 | 0.42 | 23.71 |
| full residual, sigma x 10 | 0.63 (2.70) | 0.42 0.63 2.70 0.45 2.43 | 8.53 | 8.5 7.2 11.2 8.7 6.7 | 5.28 | 0.49 | 32.81 |
| direction only, window sigma | 0.70 (2.81) | 0.46 0.70 2.81 0.70 0.59 | 8.64 | 9.8 6.7 14.0 7.3 8.6 | 3.50 | 0.65 | 27.23 |
| tilt prior 0.42 mrad, no factor | 2.78 (3.04) | 3.04 2.78 2.12 3.00 2.65 | 4.81 | 5.5 4.6 4.3 5.1 4.8 | 3.53 | 0.93 | 5.18 |
| shipped (direction only, scale 2) | 0.67 (1.85) | 0.36 0.61 0.72 1.85 0.67 | 7.83 | 8.6 5.8 12.5 7.8 7.8 | 3.53 | 0.50 | 22.05 |

**10b, gravity magnitude.** The box world, `sim/mss_sim/synth.py` (`GRAVITY_W`) and
`imu.gravity_mps2` all use 9.80665 m/s^2. For seeds 42..46, `|mean specific force of the window|
- 9.80665` is -0.0018, -0.0064, -0.0035, +0.0036 and +0.0051 m/s^2. That is within 1.6 sigma of
the 4.1e-3 m/s^2 standard error, so the window carries no magnitude mismatch.

Dropping the magnitude row (direction only) still helps the GNSS corridor over 15 seeds (10e):
the median falls from 3.72 m (full residual) to 2.44 m (shipped form, scale 1), with the same
NEES (9.60 / 9.42). The magnitude row adds nothing about the tilt and pins `ba_z` to one 2 s
noise realisation, so it is dropped.

### 10c. Seed 42 over time

| variant | t (s) | x err (m) | z err (m) | kf tilt err about w x / y (mrad) | ba_b x / z (m/s^2) | E yaw err (mrad) | E pos err (m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| main | 20.3 | +0.546 | +2.168 | -0.16 / +5.00 | +0.0091 / +0.0167 | -31.88 | 2.091 |
| main | 43.9 | +2.066 | +0.604 | -2.01 / +29.81 | +0.2378 / +0.0039 | -9.13 | 1.719 |
| main | 53.9 | +1.294 | +0.290 | -6.62 / +21.41 | +0.2729 / +0.0021 | -5.21 | 1.488 |
| full residual, window sigma | 24.4 | -5.401 | -2.168 | +1.29 / +16.78 | +0.0827 / +0.0252 | +1018.52 | 4.450 |
| full residual, window sigma | 34.6 | +0.734 | -1.918 | +32.40 / +7.56 | -0.0505 / -0.0008 | -12.81 | 2.609 |
| full residual, window sigma | 64.9 | -3.138 | -1.637 | +11.25 / -6.50 | -0.1816 / +0.0002 | +3.01 | 2.990 |
| historical round 1, full residual (pre-check) | 34.8 | -3.397 | -3.246 | +1.01 / -88.45 | -0.8345 / -0.0009 | +56.83 | 2.637 |
| shipped | 22.4 | -0.080 | -0.082 | -0.28 / -0.68 | -0.0297 / -0.0006 | -41.57 | 0.312 |
| shipped | 44.3 | +0.388 | -0.083 | +0.06 / -3.36 | -0.0405 / -0.0009 | -5.56 | 0.291 |
| shipped | 63.9 | +0.481 | -0.092 | -0.53 / -0.67 | -0.0107 / -0.0020 | -2.06 | 0.297 |

**10c, interaction with ADR-019.** The forms that anchor the tilt at the window's information
or more (full residual window sigma, direction only, shipped) end with a smaller `E` yaw error
than main over seeds 42..46: medians 2.68, 3.50 and 3.53 mrad against 5.28 mrad. The looser
or over-tight forms do not (full residual sigma / 10: 5.95; x 3: 4.50; x 10: 5.28 mrad). So the
factor does not move error into `E`'s yaw.

What it does change is which way the straight corridor's null mode goes. Along a straight run
the body x axis stays on world x, so a tilt `theta` about world y and the accelerometer bias
`ba_x` enter the IMU factors only as `g theta - ba_x`. The full residual fixes that combination
at keyframe 0, not each term, and the solution can slide along the null mode.

In seed 42 a transient yaw excursion of `E` during the early alignment (1018 mrad at 24 s, the
track is still short and straight) is followed by a slide:
- in round 1, to -88 mrad of pitch with `ba_x` -0.83 m/s^2 (`g * 0.088 = 0.87`, 8 sigma of the
  bias prior) and 3.2 m of z error;
- at the head of the branch (consistency check present), to +32 mrad of roll at 35 s instead,
  with 1.9 m of z error.

The path along the null mode is chaotic; its size is the point. Without the factor the same
mode also moves (+30 mrad, `ba_x` +0.24 m/s^2 at 44 s), less in this seed. The full residual at
the window sigma is also over-confident against the rest of the graph (`scan_match.cov_scale`
was calibrated without it): box NEES rises from 7.5 to 9.6.

**10d.** The static window's tilt as a prior (0.42 mrad) without bias coupling gives the lowest
NEES (4.8 over 5 seeds, 5.0 over 15) and a GNSS corridor at main's level. It is wrong for a
biased IMU: on the circle with a (0.05, 0.03, 0) m/s^2 bias it gives 7.5 m, against 5.2 m with
the factor (section 9).

### 10e. The forms over 15 seeds (42..56), paired against main

Five seeds were not enough: main's GNSS-corridor ATE is heavy-tailed (0.54 to 27.9 m), and seeds
42..46 were its five best. Each cell is:
- median (max) over 15 seeds;
- wins / losses against main on the same seeds;
- the 95 % bootstrap CI (2000 resamples of the seed pairs, fixed seed) of the median paired
  difference, variant - main.

Lower is better everywhere. The IMU-only rows also report the along-x RMS of every published
estimate and the largest `|x err(t)| / sigma_DR(t)` (over `T >= 2 s`), not only the end error.

| metric | main | full residual, window sigma | shipped, scale 1 | scale 1.5 | **scale 2 (shipped)** | scale 3 |
| --- | --- | --- | --- | --- | --- | --- |
| GNSS corridor ATE (m) | 3.03 (27.9) | 3.72 (32.6), 8/7, [-6.8, +3.2] | 2.44 (27.2), 9/6, [-20.2, +2.3] | 3.28 (35.7), 9/6, [-19.4, +0.6] | 1.85 (11.3), 8/7, [-20.8, +1.2] | 2.14 (34.8), 8/7, [-6.8, +1.5] |
| LiDAR-only corridor ATE (m) | 24.9 (64.0) | 31.1 (76.0), 7/8, [-15.1, +30.8] | 21.4 (73.1), 9/6, [-14.2, +5.5] | 28.9 (128.3), 9/6, [-14.0, +26.7] | 37.5 (107.1), 6/9, [-11.0, +21.6] | 26.5 (89.5), 8/7, [-11.5, +11.0] |
| box NEES (exp. 6) | 7.49 (13.3) | 9.60 (15.3), 3/12, [+0.31, +3.86] | 9.42 (15.3), 5/10, [-0.00, +4.28] | 8.49 (13.5), 5/10, [-0.51, +2.75] | 8.57 (12.5), 6/9, [-0.39, +2.09] | 7.51 (11.5), 8/7, [-0.68, +1.04] |
| circle \|end x err\| (m) | 21.7 (224) | 2.08 (8.88), 13/2 | 2.97 (6.70), 13/2 | 2.83 (14.2), 12/3 | 1.45 (7.02), 14/1, [-83.4, -0.94] | 2.15 (11.4), 12/3 |
| circle along-x RMS (m) | 11.6 (99.3) | 1.15 (3.12), 15/0 | 1.31 (3.73), 15/0 | 1.22 (5.77), 15/0 | 1.36 (2.84), 15/0, [-33.8, -4.7] | 1.87 (4.33), 13/2 |
| circle max \|x\|/sigma_DR(t) | 79.5 (349) | 6.40 (18.4), 15/0 | 6.07 (20.2), 15/0 | 5.88 (20.3), 15/0 | 7.09 (23.8), 15/0, [-148, -24] | 12.5 (22.7), 15/0 |
| straight \|end x err\| (m) | 3.56 (66.5) | 1.72 (6.92), 11/4 | 1.86 (23.6), 10/5 | 1.29 (5.62), 10/5 | 2.72 (6.28), 10/5, [-6.4, +1.2] | 4.14 (14.6), 8/7 |
| straight along-x RMS (m) | 1.85 (23.8) | 0.85 (2.63), 11/4 | 0.79 (8.88), 12/3 | 1.06 (2.58), 11/4 | 1.34 (3.07), 12/3, [-2.2, -0.17] | 1.30 (4.66), 10/5 |
| straight max \|x\|/sigma_DR(t) | 8.35 (95.7) | 4.11 (11.4), 11/4 | 4.25 (35.8), 12/3 | 6.33 (11.7), 12/3 | 6.64 (11.8), 12/3, [-10.5, -1.3] | 7.58 (20.8), 11/4 |

The consistency check (section 12) removed the factor in 0 of the 30 IMU-only runs at scales
1 to 2 and in 1 of 30 at scale 3. The tables with every CI and the per-seed values are in
`imu_round2_seeds.md`.

**Mid-run excursions (I-2).** For the shipped form the circle's end error (median 1.45 m) and
RMS (1.36 m) are close to dead reckoning, but the largest `|x err(t)| / sigma_DR(t)` has a
median of 7.1 and reaches 23.8. Where the maximum falls:

| seed | ratio | at T (s) | published tilt error then (mrad) |
| --- | --- | --- | --- |
| 42 | 21.7 | 9.7 | 11.4 |
| 43 | 23.8 | 11.1 | 9.6 |
| 55 | 14.7 | 11.1 | 7.7 |
| 56 | 17.9 | 10.4 | 8.3 |
| the 11 others | 2.5 to 10.0 | 5.6 to 17.9 | 0.6 to 4.0 |

The large excursions come mid-run (9 to 12 s), with transient tilt errors of 8 to 11 mrad. The
tilt leaves the keyframe-0 anchor through the LiDAR chain (section 11) and comes back as the
circle turns, so the end point lands near the truth. The factor fixes the end-point drift and
cuts the RMS by 8x (11.6 -> 1.36 m). It does not stop the mid-run wander, which stays up to 24
sigma_DR. That is a known gap (ADR-020).

**Choice of scale 2.** The GNSS-corridor and LiDAR-only medians are not monotonic in the scale
(GNSS 2.44 / 3.28 / 1.85 / 2.14 m at 1 / 1.5 / 2 / 3), and every CI contains 0. Within the
spread of 15 seeds, no scale is better or worse than main on the 60 s corridors: "not worse
within the spread", not "better". That includes the LiDAR-only 60 s corridor at scale 2, where
the median is higher (37.5 against 24.9 m, 6 wins / 9 losses, CI [-11.0, +21.6]).

The choice therefore rests on the box NEES and the IMU-only scenes, where the CIs exclude 0:
- scale 1 raises NEES the most (9.42; 1.5: 8.49; 2: 8.57; 3: 7.51);
- scale 3 gives back the straight corridor (median 4.14 m, 8 wins / 7 losses) and doubles the
  circle's mid-run ratio (12.5);
- scale 2 has the best circle end error (14 wins / 1 loss), a straight corridor better than
  main on RMS and maximum ratio (CIs exclude 0), and a NEES CI that contains 0
  ([-0.39, +2.09]).

`backend.static_gravity_sigma_scale: 2.0` ships.

**Known gap: the straight corridor misses 3 sigma_DR.** Its median end error at scale 2 is
2.72 m = 3.9 sigma_DR (the brief's target is 3 sigma_DR = 2.11 m), and the mid-run maximum
ratio has a median of 6.6. The component test asserts it with bounds recorded from the 15-seed
spread:
- median of seeds 42..44: |end| <= 6.3 m and RMS <= 3.1 m, the worst values of the 15 seeds;
- measured: 0.93 m and 0.94 m.

**Box NEES guard (M-8).** `LidarFactorCovarianceConsistent` checks one seed (42: 8.57) against
[3, 12]. Over 15 seeds both main (5.1 to 13.3) and the shipped form (4.7 to 12.5) leave that
band on some seeds, so the single-seed guard is fragile in both directions.

### 10f. Integration and bench, shipped form

`tools/run_integration.py` on Windows (Python with rosbags), bench built in WSL, scenario
overlay (`--path-style wsl --out results/imu4d-fr1 --plots`): every test passes. Main
reference: a fresh Release `mss_bench` built from the main worktree at `cc642be`, run on the
same datasets with main's `config/default.yaml` and the same `config/sim/warehouse.yaml`.

| test | main ATE (m) | branch ATE (m) |
| --- | --- | --- |
| E-01 | 0.002227 | 0.002181 |
| E-02 | byte-identical rerun | byte-identical rerun |
| E-03 | 0.002139 | 0.002175 (+1.7 %, gate 20 %) |

The factor is added in E-01..E-03 (`n_gravity_factor_gated` 0) and stays
(`n_gravity_factor_removed` 0). A first run with `imu.init_max_wheel_speed_mps` 0.01 gated it:
the still phase's wheel speeds exceed 0.01 m/s from encoder quantization and noise, so the
bound is 0.1 m/s (`wheel_odom.speed_sigma_mps` 0.02, 5 sigma).

E-01 bench, median of 5, interleaved, main / branch: scan p50 4.18 / 4.19 ms, p95 5.88 / 5.61
ms, `replay_wall_s` 2.52 / 2.48 s, CPU 100 / 100 %.

The round-2 p95 rise (+5 to 9 %) is not reproduced. Splitting the per-scan time in process
(`ImuStaticGravityLatencySplit`, median of 5, 90 keyframe scans, factor off / on):

| scans | p50 (ms), off / on | p95 (ms), off / on |
| --- | --- | --- |
| keyframe | 4.76 / 4.72 | 6.28 / 5.76 |
| non-keyframe | 4.15 / 4.15 | 5.58 / 5.53 |
| all | — | 5.74 / 5.65 |

So the factor adds no per-scan or iSAM2 cost; the round-2 difference was run-to-run variation
of the separate bench processes.

## 11. Tilt wander over long runs

60 s corridor, LiDAR + IMU (relax 1e8), no GNSS, seeds 42..46, medians. Columns:
- backend tilt error (about world x and y, norm) at 10 / 20 / 40 / 60 s;
- RMS per axis of the tilt part of the LiDAR factors' rotation error;
- the random walk those errors would give if independent (`rms sqrt(N)`).

| variant | tilt err at 10 / 20 / 40 / 60 s (mrad) | factor tilt err RMS (mrad) | factors | rms sqrt(N) (mrad) | ATE (m) |
| --- | --- | --- | --- | --- | --- |
| main (factor off) | 2.46 / 3.60 / 4.26 / 13.57 | 8.0 | 58 | 61.0 | 21.56 |
| full residual, window sigma | 2.56 / 2.66 / 6.17 / 5.83 | 5.3 | 42 | 34.2 | 28.45 |
| direction only, window sigma | 2.30 / 1.94 / 5.21 / 6.65 | 4.3 | 36 | 25.8 | 20.30 |
| shipped (direction only, scale 2) | 1.87 / 1.45 / 2.87 / 1.96 | 6.6 | 20 | 29.6 | 15.34 |

The LiDAR factors' tilt errors (4 to 8 mrad each) would random-walk to 26 to 61 mrad if they
were independent. The observed tilt wander is 2 to 14 mrad: the scan-to-map matches re-anchor
on the shared map, so consecutive errors largely cancel. The wander grows once the map window
(`map.window_keyframes`, 10 m range here) forgets the early keyframes. Main reaches 13.6 mrad at
60 s, about 24 m of gravity leak over 60 s (`0.5 g theta T^2`, with theta ramping), the order of
its 21.6 m ATE.

With the shipped form the median keyframe tilt is 2.0 mrad at 60 s over these five seeds. Over
15 seeds (10e) its LiDAR-only ATE median is 37.5 m against 24.9 m for main, 6 wins / 9
losses, CI [-11.0, +21.6]: within the spread, but the factor does not make the long LiDAR-only
run better.

A periodic re-anchor is the follow-up: a weak gravity-direction factor on the keyframe in each
later window the `imu.init_*` thresholds and the rest gate accept as static. It does not help
in these corridors, which never stop, so it is not implemented here.

## 12. Rest precondition: rest gate and consistency check (fix round 1, I-1)

The factor reads the init window as "body at rest". The initializer accepts a window on
per-axis std only, so a constant-rate turn (centripetal 0.125 m/s^2 at 0.5 m/s on R = 2 m, a
12.7 mrad tilt read into the window) or a steady acceleration passes. At the factor's 0.84 mrad
that would be pinned at about 15 sigma.

Two guards (keys under `imu` and `backend`, validated by `load_config`):

**Rest gate, before the factor is added** (`gravity_rest.hpp`, logged and counted in
`Stats::n_gravity_factor_gated`). Each check, its bound and why:

| check | bound | why |
| --- | --- | --- |
| mean gyro | `\|mean gyro\|` <= `imu.init_max_mean_gyro_radps` | 0.15 rad/s: 4.3 sigma of the sim profile's gyro bias (0.035 rad/s); a 0.25 rad/s turn fails |
| wheel speed (when `wheel_odom.enabled`) | <= `imu.init_max_wheel_speed_mps` over the window | 0.1 m/s, above the noise at rest (10f) |
| specific-force magnitude | `\|\|f\| - g\|` <= `imu.init_gravity_check_sigma` x sqrt(u^T C u + prior_bias_acc^2) | factor 3; weak, since a horizontal acceleration changes `\|f\|` only to second order |

**Consistency check, after keyframe `backend.static_gravity_check_keyframes` (5).** If the
factor's chi-square exceeds `backend.static_gravity_chi2_gate` (5.991, chi-square 2 DoF at 0.95),
it is removed from iSAM2 (`removeFactorIndices`), counted in `Stats::n_gravity_factor_removed`
(and `bench.json`), and one warning is logged.

The check needs keyframes: 10, the value first proposed, was never reached in a 20 s
slow-start run (8 to 9 keyframes), so it is 5. At rest its false-removal rate is the gate's 5 %
in theory; it removed the factor in 0 of 90 IMU-only runs at scales 1 to 2 (section 10e) and in
none of E-01..E-03.

Component tests, `PipelineComponent.GravityFactor*`, box world without the still phase,
LiDAR + IMU:

| start | factor off ATE (m) | shipped ATE (m) | what happened |
| --- | --- | --- | --- |
| circle from t = 0 (0.25 rad/s turn) | 1.0017 | 1.0017 | gated (mean gyro) |
| same, gyro bound opened to 1 rad/s | 1.0017 | 1.0006 | added, chi-square 43.0 at the check, removed |
| straight, 6 s speed ramp from t = 0 (mean a 0.06 m/s^2 in the window) | 0.1043 | 0.1025 | added, chi-square 28.9, removed |
| straight, 2 s speed ramp (mean a 0.25 m/s^2) | 0.5465 | 0.5575 (+2 %) | added, chi-square 207, removed |

The tests assert that the factor is gated or removed and that the ATE is at most 1.1x the
factor-off ATE (the 1.0001x in the gated case). A start in motion stays a start in motion: the
ATE without the factor is already 0.1 to 1 m, because the initial velocity is taken as zero.

## 13. What the shipped factor fixes, and what it does not

**Fixes (15 seeds, median):**
- the IMU-only corridor's divergence: circle end 21.7 -> 1.45 m, RMS 11.6 -> 1.36 m; straight
  end 3.56 -> 2.72 m, RMS 1.85 -> 1.34 m;
- E-01 / E-03 within 2 %; the GNSS corridor not worse within the spread (1.85 against 3.03 m,
  CI contains 0).

**Does not fix:**
- mid-run excursions of up to 24 sigma_DR(t) (circle median of the maximum 7.1) with transient
  tilt errors of 8 to 11 mrad;
- the straight corridor's 3 sigma_DR target (median 3.9 sigma_DR);
- the long-run tilt wander through the LiDAR chain (60 s LiDAR-only corridor, median 37.5 m
  against 24.9 m, within the spread);
- the backend consistency defect of the exploratory controls (`BetweenFactor<Pose3>`'s
  approximate Jacobian, x-removed no-feedback control at +5772 m with sigma 14 m). That is
  ADR-015's note and PR 4e.
