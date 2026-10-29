# Lessons learned

Problems met while building this system, in the order they surfaced, with the cause, the fix
and the rule that would have avoided them. Each entry names the PR, ADR or derivation that
holds the numbers. The list exists because every one of these problems is generic: a new
LiDAR-inertial system on a factor graph will meet them again.

Format of an entry: **symptom** → cause → fix → rule.

## 1. Estimation

### 1.1 An unobserved axis drifts far beyond inertial dead reckoning (PR 4d, ADR-020, `derivations/imu_only_observability.md`)

- **Symptom.** In a corridor whose end walls are out of LiDAR range, LiDAR + IMU drifted 5 to
  97 m along the corridor in 17 to 20 s. Inertial dead reckoning with the same IMU predicts
  about 0.6 m (1σ). The keyframe count rose with the drift (23 → 28 → 99 keyframes), which
  is the first visible sign.
- **Cause.** With world x unobserved, a tilt θ about the horizontal axis normal to x is
  indistinguishable in the IMU factors from an acceleration g·θ along x. The only absolute
  tilt anchor was the keyframe-0 prior at 1° (17.5 mrad), while the static initialisation
  window had measured the tilt to 0.42 mrad. The backend's tilt is written into the map through
  keyframe insertion and rebuilds, so the map re-anchors the LiDAR to the wrong tilt and the
  error compounds (tilt reached 150 to 212 mrad in the controls). Front-end feedback,
  relinearisation, the bias prior and the keyframe rule were tested one by one: amplifiers,
  not the cause.
- **Fix.** A gravity-direction factor on keyframe 0 built from the static window's mean
  specific force and its covariance, tying tilt and horizontal accelerometer bias together
  the way the window does (a bare tight tilt prior pins the wrong tilt when the IMU has a real
  bias). Circle end error 21.7 → 1.45 m, straight 3.56 → 2.72 m over 15 seeds.
- **Rule.** Every absolute anchor placed at initialisation must carry the information the
  initialiser actually measured, in the same parametrisation. A loose prior on a mode that
  nothing else observes is not "safe": it is the mode the whole solution will drift along.
  Instrument the keyframe count; a rising count on a straight run means drift.

### 1.2 A consistency self-check fires at random when the graph's covariances are not calibrated (PR 4d, `derivations/imu_only_observability.md` §10g)

- **Symptom.** A chi-square(2) check meant to remove the gravity factor after a start in
  motion removed it in 14 % of runs that started at rest (40 % in the GNSS corridor). Its
  statistic at rest (37 to 147) overlapped its values at true moving starts (38 to 233), so no
  threshold separates the cases.
- **Cause.** The rest of the graph is over-confident at the check keyframe (LiDAR NEES 8.6
  against the expected 6; the GNSS frame variable still aligning). A hypothesis test built on
  miscalibrated covariances has an unknown false-alarm rate. Using the innovation statistic
  instead of the posterior residual did not change the rate.
- **Fix.** The check ships off; the rest gate on the initialisation window (gyro, wheel speed)
  stays. A wrong gravity factor at a straight-acceleration start costs at most 2 % ATE because
  the initialiser already uses the same window mean for the initial attitude.
- **Rule.** Calibrate NEES of every factor before adding any self-check that depends on it.
  Measure a check's false-alarm rate on at-rest data across many seeds before shipping it.
  Compare the cost of the failure the check prevents with the cost of its false alarms.

### 1.3 A frame alignment estimated once and frozen becomes a bias (PR #19, ADR-017 → ADR-019, `derivations/gnss_fusion_study.md`)

- **Symptom.** Adding GNSS at 0.5 m sigma made the box-world ATE 3.4 to 3.7× worse than
  LiDAR-only. The first proposal (withhold GNSS factors until the aligner's spread is large)
  helped only while it never opened, and in a corridor it delayed alignment until the chain
  had drifted.
- **Cause.** The ENU-to-world transform `T_e_w` from the aligner was slightly wrong and fixed;
  every GNSS factor then pulled the loosely anchored start pose and heading through that
  wrong frame.
- **Fix.** `T_e_w` is one `Pose3` variable in the graph shared by all GNSS factors, seeded from
  the aligner with configurable priors. Box ATE with GNSS 0.0322 → 0.0090 m (1.00× LiDAR-only);
  E-01 0.0709 → 0.0022 m.
- **Rule.** Any transform estimated from data at start-up (sensor-to-world frame, extrinsic
  refinement, time offset) is a variable, not a constant. If it must be frozen, bound its
  error and show the factors that use it tolerate that error.

### 1.4 A rejection gate with no escape path locks out the sensor for good (PR #19)

- **Symptom.** After a 20 s GNSS outage, every later fix failed the chi-square gate and no
  GNSS factor was ever added again, because the estimate had drifted and the gate compared
  new fixes against a drifted prediction.
- **Fix.** After `gnss.max_consecutive_rejects` consecutive rejections the aligner restarts
  (counter `n_gnss_realign`). The guard length (60 fixes = 12 s at 5 Hz) is set longer than the
  longest multipath hold in the simulation (10 s) so biased fixes are not re-admitted.
- **Rule.** Every gate needs a documented way back in, and the way back in must be argued
  against the longest plausible run of genuinely bad measurements.

### 1.5 Factor covariances are over-confident until measured (PR #19, `scan_match.cov_scale`)

- **Symptom.** LiDAR scan-to-map NEES was 28 against an expected 6.
- **Fix.** `cov_scale` 4.7, chosen from the measured NEES (28 → 7.8); a test keeps NEES in
  [3, 12]. Both main and later branches still leave that band on some seeds, so the band is a
  guard, not a proof.
- **Rule.** Report NEES per factor type on a known-answer scene before fusing it with anything
  else. Treat a scan-to-map covariance (absolute information) re-used as a relative factor with
  suspicion.

### 1.6 Degeneracy detection is defeated by small spurious information (PR 4b, `derivations/degeneracy.md`, `scan_match.md`)

- **Symptom.** Eigenvalue-ratio detection needed scene-specific thresholds (Schur relative
  mode needed a ratio of 0.03 with a 1.3× margin and produced roll false positives). A
  sigma-based criterion could not flag the corridor axis because 1.15 % of the information
  along it leaks from wall–floor and wall–ceiling edge neighbourhoods, which is enough for a
  3.8 mm posterior standard deviation.
- **Rule.** A direction is only "unobserved" if the leak is below the noise floor; measure the
  leak (information along the axis as a fraction of the dominant axis) before designing the
  detector. Prefer absolute sigma thresholds over ratios, and expect the map's edge
  neighbourhoods to supply false information.
- **Status (PR #29).** The sigma detector is held on a branch (PR 4b-2), not merged: it flags
  nothing in the corridor until the map edge leak is fixed. Only the absolute detector and the
  "unobserved" covariance (PR 4b-1) are on main, with the detector off by default.

### 1.7 Map storage policy creates registration bias (PR #20, ADR-007 amendment, `derivations/map.md`)

- **Symptom.** A 9.5 mm / 0.08° bias in the S-02 known-answer test at the shipped
  configuration.
- **Cause.** A ring of 20 points per 0.5 m voxel filled from consecutive scans at nearby poses
  puts 32 candidate points into 20 slots on faces at voxel boundaries, producing two nearly
  collinear point columns on two faces; a 5-NN neighbourhood of such columns is exactly
  coplanar with a diagonal normal.
- **Fix.** Minimum point spacing in a voxel (replace-nearest), evaluated and kept as an
  option: it removes the bias (0.38 mm) and 29 % of map points but costs E-01 LiDAR-only ATE
  in the first sparse seconds.
- **Rule.** A map is a sampling policy. Test the map's point distribution on a planar face and
  an edge before tuning the matcher.

### 1.8 A gate that looks good on a noise-free simulator is not evidence (PR #20)

- **Symptom.** A planarity gate at ratio 0.01 improved E-01 from 2.20 to 0.95 mm on the
  noise-free simulator and removed 90 % of the corridor-axis leak.
- **Cause.** At 1 cm range noise about half of all planar 5-NN patches fail the same gate
  (Monte-Carlo); on a replay with default extrinsics ATE went 14.8 → 24.0 mm and p95 latency
  6.1 → 11.5 ms.
- **Rule.** Evaluate every geometric gate with the sensor noise model on, and prefer a
  noise-aware form (`λ_min > ρ·λ_mid + n·(k·σ_range)²`) over a pure ratio.

### 1.9 The residual's chart, not only its Jacobian, decides a free axis (PR 4e, ADR-021)

- **Symptom.** In a control run with the LiDAR constraint removed along one axis the estimate
  ended at +5772 m while the marginal sigma claimed 14 m.
- **Cause.** Two effects. GTSAM's `BetweenFactor<Pose3>` approximates the `Logmap` derivative
  (no `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR`), which is wrong at metre residuals. The dominant
  one is the SE(3) residual itself: `rho = V(phi)^-1 t` couples a large residual along the
  unobserved translation axis into the observed components, which then count as information.
  With the exact Jacobian (`se3_exact`) the divergence goes, but the estimate stays 16 sigma
  over-confident (-28 m, sigma 1.75 m). Translation and rotation residuals kept separate
  (`decoupled`, the SO(3) x R3 chart, the same covariance to first order) keep the free axis
  free: -8.7 m, sigma 24.5 m.
- **Mechanism.** The SE(3) Log residual's translation row is
  `rho = V(phi)^-1 t ~= t - 1/2 phi x t`. With a metre-scale `t` along the free axis, the term
  `1/2 [t]_x` is a lever arm: it moves the rotation perturbation into the translation rows,
  which are weighted by the centimetre translation sigma. The well-observed rotation then
  leaks into the free translation axis as information it does not have. The scan matcher's
  covariance was computed for translation and rotation read separately, so only `decoupled`
  uses it in its own chart. `between` also uses the approximate `Logmap` derivative, which is
  why it diverges instead of being only over-confident.
- **Fix.** `backend.relative_factor` (`between`, `se3_exact`, `decoupled`); default `decoupled`
  (ADR-021). `between` and `se3_exact` remain selectable for comparison.
- **Rule.** An exact Jacobian is necessary but not sufficient. A covariance is only meaningful
  in the chart it was computed in; at large residuals the SE(3) Log chart and the SO(3) x R3
  chart diverge. State the chart of every covariance that crosses a module interface, and test
  factors on a free axis at metre residuals, not only at the measurement. Check every analytic
  Jacobian against numerical differences at small and at large residuals. Read the build flags
  of a dependency before trusting its defaults.
- **Same defect in `PriorFactor<Pose3>` (PR #28).** `gtsam::PriorFactor<Pose3>` also
  returns an identity Jacobian for its `Logmap` error, on keyframe 0, the re-anchor priors and
  the GNSS frame variable `E` (whose prior allows tens of degrees of yaw). It was found only
  because B-04 compared two optimisers that accept steps by different criteria: LM by the true
  cost, Gauss-Newton and iSAM2 by the linearisation; they disagreed by up to 1.8e-6
  (velocity), above the 1e-6 limit. Fixed by
  `PosePriorFactor` (same residual, exact Jacobian). Rule: audit every library factor in the
  graph, not only the one that failed; a disagreement between two optimisers at 1e-6 is a
  signal, not noise.

### 1.10 Failure paths in the optimiser must re-anchor (PR #11)

- **Symptom.** A failed `optimize()` left an island of keyframes with no prior, and a NaN
  initial guess reached the optimiser.
- **Fix.** Pose prior on orphaned keyframes after a failed optimisation; all inputs sanitised
  (`values_finite`) before and after every update, including after factor removal.
- **Rule.** Each `optimize()` failure path gets a test, and the graph after any removal or
  rebuild is checked for connectivity and finiteness.

### 1.11 End error hides mid-run excursions (PR 4d)

- **Symptom.** The IMU-only circle ends within 1.45 m but reaches 24× the dead-reckoning sigma
  at 9 to 12 s before the map pulls it back.
- **Rule.** Report RMS and maximum ratio over the run, not only the end error.

### 1.12 Single-seed tests are fragile both ways (PRs #19, 4d, #29)

- **Symptom.** Five seeds happened to be the baseline's five best; a single-seed NEES band
  was left by both the baseline and the candidate on other seeds; corridor ATE tails are
  heavy (median 3.0 m, mean 9.3 m, max 27.9 m over 15 seeds). The #19 corridor test pinned one
  outcome, and a rounding-only change in the unobserved block (`U = P_F D P_F`,
  basis-invariant) moved seed 42 at 1000 m^2 from 1.78 to 0.51 m (PR 4b-1, #29).
- **Cause.** The scene is chaotic in the variance, the seed and the rounding
  (`derivations/degeneracy.md` section 7), so a single seed's value is not a property of the
  code (see 1.15).
- **Rule.** Decide defaults on 15 or more paired seeds with wins/losses and a bootstrap CI of
  the median difference. A bound taken from the worst seed is a regression guard, not evidence
  for a feature; say which in the test comment. A test over a chaotic scene asserts ratios to
  a reference run (fused / LiDAR-only) and the median over several seeds, never the value of
  one seed.

### 1.13 Tilt wanders once the map window forgets the early keyframes (PR 4d, open)

- **Symptom.** With the gravity factor on, the backend tilt error is 1.5 to 2.9 mrad for the
  first 40 s and grows afterwards; each LiDAR factor's tilt error is 5 to 8 mrad.
- **Status.** Periodic gravity re-anchoring at static windows is the planned follow-up; the
  current test corridors never stop, so it cannot yet be evaluated.
- **Rule.** Decide how every absolute quantity (gravity direction, heading, scale) is
  re-anchored over time, not only at start.

### 1.14 A covariance floor can make a new feature equal to the old one (PR 4b-1, #29, `derivations/degeneracy.md` section 7)

- **Symptom.** On the recorded Gazebo corridor (E-04) the "unobserved" covariance at the default
  0.001 m^2, at 1e-4 m^2 and at 1e-12 m^2 all gave the pre-4b relaxed covariance.
- **Cause.** The variance is floored at `relax_factor * c / eig_threshold`, about 1.2e-3 m^2
  with the Gazebo residuals, so below that value the floor binds and the setting does nothing.
- **Fix.** Report the floor next to the configured value; sweep above it (0.01 m^2: 0.41 m,
  0.1 m^2: 0.34 m, 1 m^2: 1.26 m on that one bag); keep the default (owner decision) and retune
  over several recordings in S2 (candidate 0.01 m^2).
- **Rule.** Before comparing a new feature with the old one, check whether a floor, clamp or
  gate makes them identical at the tested setting.

### 1.15 One chaotic run cannot rank two options (PR 4b-1, #29, `derivations/degeneracy.md` section 7)

- **Symptom.** On the same E-04 bag, main and the old semantics were bit-identical up to
  t = 5.7 s, differed at the 1e-13 level, and ended at 1.69 m against 2.20 m ATE; runs with the
  same semantics spread from 1.8 to 3.4 m.
- **Cause.** With degeneracy on, the gate that rejects GNSS fixes turns tiny differences into
  different histories.
- **Fix.** No value was chosen from one bag; the options are compared over several
  recordings (S2).
- **Rule.** Rank options on several runs (seeds or recordings), never on one chaotic run; a
  difference that grows from 1e-13 to metres is the signature. See 1.12.

### 1.16 Choose the initialisation method from the sensors the platform actually has (PR #34, ADR-023, `derivations/gnss_fusion_study.md` section 15)

- **Symptom.** The GNSS frame yaw from the track direction (design 2) flipped 180° in review
  probes in two cases:
  - on a fast turn (above about 1.57 rad/s with 10-fix chunks);
  - whenever the robot's net motion was backward before the yaw was confident (3 m forward then
    back: -177°).

  A fix covariance declared as zero, or a parked robot with slowly drifting GNSS, aligned with an
  error up to 180° and a small reported sigma.
- **Cause.** The brief proposed the method without checking it against what the hardware gives.
  - The usual sources of absolute heading are a magnetometer (9-axis IMU), GNSS Doppler
    velocity, a dual-antenna receiver, or the course over ground of a moving vehicle.
  - The MPU IMU has no trustworthy magnetometer, and the simulated receiver gives positions only,
    so course over ground was the only option.
  - Course over ground has known preconditions: a sign of travel, a minimum speed or track
    length, straight segments, and an honest noise floor.
  - The first design took the sign of travel from a forward-driving assumption. The brief's
    alternative, the front end, is exactly the unobserved axis that broke the old aligner.
- **Fix.** Use the standard land-vehicle practice with the sensors present:
  - The sign of travel comes from the wheel-odometry speed, per segment. The forward-driving
    assumption is used only without wheel data, and those segments are counted. A segment with
    less than 0.2 m of wheel travel is stationary and is skipped: wheel noise at rest has a
    random sign, and using it cut E-01 from 65 GNSS factors to 8.
  - Segments are excluded when the heading change exceeds 45° (`gnss.align_yaw_max_turn_deg`).
    The heading is unwrapped so that the turn is measured correctly.
  - The lever-arm term depends only on rotation, and is solved in closed form.
  - The declared fix variance is floored at 0.25 m², and non-positive covariance is rejected.
  - Alignment requires a minimum track length of 5 m and a track coherence of 0.5. Coherence is
    the track along the heading divided by the GNSS path length. With a GNSS random walk of
    0.05 m per fix, a parked robot aligned on 13 of 100 seeds before and on 0 after.
  - That protection has an envelope. Without wheel data the parked robot aligns on 0 of 100
    seeds at 0.10 m per fix (3 of 100 in a review probe with other seeds, up to 154° off) and
    on 16 of 100 at 0.20 m per fix (up to 176° off). With wheel data it aligns on 0 of 100 at
    every level. The IMU rest detector cannot help, because rest and constant-velocity driving
    look the same to an IMU.
  - With wheel data, a robot slower than about 0.1 m/s never aligns at the defaults: every
    segment counts as stationary.

  Two cases remain without wheel data: net backward motion before the yaw is confident, and
  a parked robot whose GNSS drifts by 0.1 m per fix or more. Both are documented as
  limitations in ADR-023 and pinned by tests.
- **Rule.** Before designing an estimator step, list the sensors the target platform has and
  what each one observes. Then compare with how established systems solve the same step and
  which sensors they assume. Pick the method whose preconditions your hardware can check. Never
  take a decisive quantity (a sign, a scale, an initial yaw) from a state that the diagnosis
  already showed is unobserved.

### 1.17 Wait for observability by a condition, not by skipping frames (PRs #33 and #34, ADR-022, ADR-023)

- **Symptom.** The GNSS frame yaw can't be found while the robot is still: the fixes are a noise
  cloud around one point. Skipping the first N frames until the heading "settles" looks like the
  easy fix. It also matches the way a particle-filter localiser waits for some motion before it
  trusts its pose.
- **Cause.** Yaw becomes observable only with motion: enough track length on straight segments.
  - A fixed N is not tied to that. A robot that stays still for longer is still unaligned after
    N frames, and a fast one waits for nothing.
  - Skipping frames also throws away good data. The LiDAR-inertial front end is correct in the
    local frame from the first frame; only the local-to-ENU transform is unknown.
- **Fix (chosen).**
  - The front end runs and publishes the local pose from the first frame.
  - The only fixed wait is the static phase at keyframe 0, for IMU bias and gravity.
  - The GNSS alignment waits on conditions, all of which must pass:
    - predicted yaw sigma at most 3° (6° since design 2b, section 1.18);
    - at least 5 m of track along the heading (3 m when every segment has wheel data, 1.18);
    - track coherence at least 0.5;
    - segments that turn more than 45° are not used (with wheel data they take their direction
      from the wheel path since design 2b);
    - segments without wheel motion are not used.
  - Fixes received before alignment are buffered and attached once it succeeds (design 1), not
    dropped. The buffer is sized to the expected alignment delay (320 fixes; E-01 needs 264).
  - The global (ENU) output is to carry an "aligned" flag. It is valid only after alignment, and
    consumers use the local pose until then (planned for S2).
- **Rule.** When a quantity needs excitation to become observable, gate on a measure of that
  observability (a sigma, a track length, an eigenvalue), never on a frame count. Keep the data
  that arrives before the gate opens, and say in the output which estimates are valid yet.

### 1.18 Hardware numbers decide which alignment motion works (design 2b, ADR-023 amendment, `derivations/gnss.md` section 5.1, `derivations/gnss_fusion_study.md` section 16)

- **Symptom.** The GNSS frame yaw needed about 6 m of straight driving before it aligned (box
  world 6 to 17 GNSS factors instead of 38 to 45, E-01 35 of 223). Several ways to align with
  less motion were proposed: rotate in place, use turns, trust the front-end displacement.
- **The owner's numbers.** Standard (non-RTK) GNSS; 0.5 m per axis is the working value in the
  simulator. Antenna lever arm 0.22 m horizontal (`p_b_g_m` [-0.20, 0.10, 0.60]). Wheel odometry
  on the robot.
- **What the numbers say.**
  - **Rotating in place.** The antenna circles the body origin at 0.22 m, so the yaw sigma is
    `sigma / (|l| sqrt(N))`: one rotation in 10 s at 0.5 m gives 19.5 deg at the simulator's
    5 Hz (Monte Carlo; 18.1 predicted) and 12.9 deg at 10 Hz (12.8). With RTK at 2 cm the same
    rotation gives 0.5 deg at 10 Hz. With
    GNSS error correlated over a few seconds it is several times worse (46 deg). Useless
    on this hardware, good with RTK.
  - **Straight driving.** On a straight track `sigma_yaw = sqrt(2 v) / L` with `v = sigma^2 / 10`
    per 2 s chunk: `L = 0.224 m / sigma_yaw`, so 4.3 m for 3 deg, 2.8 m for 4.5 deg, 2.1 m for
    6 deg between the first and last chunk centres (probe with no minimum track, 200 seeds at
    0.5 m/s: first alignment at 4.9, 3.9 and 2.9 m driven; study section 16.1). The 5 m minimum
    track, not the sigma, set the distance at 3 deg.
  - **Turns.** The chord between two chunk means on a turn bends with the speed inside the turn,
    by up to half the turn angle, so course over ground along the mean heading is skipped above
    45 deg. With wheel data the path shape in w is known (the odometry frame's travel along the
    gyro/LiDAR heading, with `T_b_o`: ignoring a 0.2 m IMU-to-axle offset cost 12 deg on a 1 m
    circle) and the chord direction comes from it, with a stated slip model. On aisles with
    90 deg corners at 3 deg: 2 of 200 runs aligned before, all now (median 7.9 m); a 1 m circle
    (57 deg per chunk) went from never to 12 s at 6 deg.
  - **Minimum track with wheel data.** Wheels can turn while the body stands still (stuck,
    lifted), so "the odometer detects a parked robot" is only true when the wheels stop too. A
    zero minimum aligned such a robot on GNSS drift (64 of 100 at 0.2 m per fix, about 90 deg
    off). Comparing the GNSS track with the wheel travel over a window since the last mismatch,
    and replacing all segments only after 10 m of agreeing travel, brings it to 2 of 100 (test
    seeds; 4 of 100 on the review probe's seeds), and the alignment after the wheels spun waits
    about 10 m instead of the spun distance (300 s of spinning: 150 m). The 3 m minimum was
    chosen from the absolute-error tails at 6 deg, not from NEES, which a larger sigma lowers
    while the tails grow.
  - **The frames and the time between fixes matter as much as the noise.** The direction of
    travel is the odometry frame's x axis, not the IMU's (a 90 deg mount gave a 90 deg error),
    and a GNSS outage can hide two opposite turns whose net heading change is zero (aisles: 100
    of 100 runs beyond 3 sigma, in design 2 too); the turn gate now uses the total heading
    variation from the front end between fixes.
  - **The front-end displacement.** Its covariance does not say where the LiDAR is blind in the
    shipped configuration: 1.5 to 4.6 cm along the corridor axis while the front end was 1 m off
    over 2 s. Not used.
- **Chosen (proposed; the change of the first-factor target is the owner's call).** Sigma 6 deg
  (the graph refines the yaw; the first yaw stayed within its sigma, no flip, no more rejections,
  and a forced 18 deg start converged), turns from the wheel path, wheel/GNSS consistency, 3 m
  minimum track when every segment has wheel data. With wheel data: corridor 6 -> 4 m, box 7 ->
  4 m, E-01 23 -> 9 m to the alignment. Without wheel data the corridor is unchanged (the 5 m
  guard for a parked robot stays).
- **Waiting is cheap.** The local LiDAR-inertial pose is valid from frame 0; only the ENU output
  waits for the alignment. A later alignment costs nothing for local navigation.
- **Rule.** Before proposing a manoeuvre or a source for an observability problem, compute its
  information from the sensor noise and the geometry (here `sigma / (|l| sqrt(N))` against
  `sigma sqrt(2 / n) / L`), and check that the signal you would trust is calibrated in the
  configuration that ships (the scan-match covariance was not). Measure the remaining levers in
  that order: threshold, minimum guards, then the data the gates discard. Judge a guard by the
  absolute-error tails and the failure cases it exists for (stuck wheels, drift), not by NEES.

## 2. Simulation and evaluation

### 2.1 Coupled calibration errors (PR #21, ADR-011 §3)

- **Symptom.** The planned wheel-calibration errors gave a yaw drift of 9.1°/rev instead of
  the stated 0.9°/rev.
- **Cause.** Yaw scale is `(r_b/r_t)(track_t/track_b)`: radius and track errors couple, and
  the first arithmetic ignored the radius factor. The implementer caught it.
- **Rule.** Write the formula next to every simulated error, verify it with a consistency test
  (`test_sim03_consistency` ties `config/sim/*.yaml` to `sim/noise/default.yaml`), and have the
  person who implements a number re-derive it.

### 2.2 Determinism needs the whole stack (PR #17, ADR-018)

- **Symptom.** `deterministic: true` did not give bit-identical runs: GTSAM uses TBB.
- **Fix.** A process-lifetime `tbb::global_control` limiting parallelism to 1 when the backend
  is configured single-threaded; the exported CMake config declares the TBB dependency.
- **Rule.** List every library that spawns threads and show the bit-identical test passes
  with each one constrained.

### 2.3 Hot-path allocations arrive through the side door (PR #13)

- **Symptom.** A heap allocation on the GNSS path and an epoch mismatch (pose at fix time
  versus pose at the latest keyframe) were found in review, not by tests.
- **Rule.** Allocation tests per message path, and a test that the measurement is paired with
  the state at the measurement's time.

### 2.4 A wheel collision shape sets the effective track (PR #26, `sim/README.md` "Wheel contact calibration")

- **Symptom.** Wheel odometry from the first Gazebo recording, computed with the true
  kinematics, showed a straight 10 m at -0.04 % but a spin in place of -36.25 deg/rev, so the
  odometry yaw drifted by a large amount.
- **Cause.** The wheels collided as cylinders 0.04 m wide, which touch the floor at a rim: an
  effective track of about 0.36 m instead of 0.40 m. The error scales with the width (0.01 m
  wide: -9.25 deg/rev).
- **Fix.** Wheel collision as a sphere of the wheel radius (`wheel.collision_shape: sphere`;
  the visual stays a cylinder): -0.01 % straight, -0.08 deg/rev spin (+0.00 deg/rev on the E-01
  lap). The footprint widens to y = +-0.30 m, so every route keeps 0.5 m from the world boxes
  (tested).
- **Rule.** Check the contact geometry of every wheel in a simulator against the odometry model
  with a straight-line test and a spin test, using the true kinematics, before trusting any
  odometry number.

### 2.5 The renderer error of a simulated LiDAR is not the configured noise (PR #26, `sim/README.md` "LiDAR render error")

- **Symptom.** The noise report's LiDAR range sigma per cell differed from the declared
  0.02 m, and no formula fitted: a constant, `k r`, `k (1/|n.u| - 1)` and `k tan(theta)` all
  left residuals of about 0.01 m against bin errors of 1e-3 m or less.
- **Cause.** gpu_lidar adds its own error on top of the configured noise (depth-texture
  interpolation), and it depends on range, incidence and the geometry of the world. Binning
  the calibration on the measured range makes it worse: it selects points on their own error
  at a crowded bin edge (the corridor cell 0.5-2 m x 0.3-0.5 read -14 % binned on the measured
  range and +4.7 % on the noise-free one).
- **Fix.** A measured table per world (range bin x incidence bin; 6 cells for the warehouse,
  14 for the corridor), calibrated along the scenario's own trajectory with the Gazebo noise at
  0, silhouette points excluded (a grid neighbour more than `lidar.edge_jump_m` off the plane,
  no return, or dropped), and binned on the noise-free range to the plane. Expected sigma per
  cell is `sqrt(sigma_declared^2 + sigma_render(cell)^2)`, gated at +-15 %.
- **Rule.** Calibrate a simulator's sensor error where it is used (world and trajectory) with
  its noise set to 0, exclude silhouette points, and bin on the noise-free quantity.

## 3. Process

- **Parked findings become main's defects.** Items parked after review (the BetweenFactor
  Jacobian, the IMU-only drift) were later load-bearing for other PRs. Rule since PR #16:
  every finding and known limitation is fixed before merge, or the owner decides explicitly.
- **Diagnosis before fix.** Each estimation fix above was preceded by a committed study
  (`MSS_*_STUDY=1` harness) that isolated one candidate per experiment. The fix proposed in
  the brief was wrong twice (alignment gate, tight tilt prior); the study showed why.
- **Defaults by evidence, flags by default.** Every change ships behind a key that defaults to
  the previous behaviour, then the evaluation sets the default, with the numbers next to the
  key.
- **Keep the integration gate honest.** E-01..E-03 run on every PR with the scenario overlay;
  baselines change only in a dedicated PR with before/after plots.
- **Repository hygiene.** No personal data or machine paths in tracked files (found in
  `package.xml` maintainers and plan documents); scenario constants belong to the scenario
  YAML, not to a script.
- **Parallel branches.** Squash merges of parallel module branches conflict on shared
  CMake lists; rebase with union resolution, de-duplicate, build, then force-push, and re-open
  the PR if CI does not trigger.
