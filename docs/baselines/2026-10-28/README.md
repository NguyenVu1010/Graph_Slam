# First baseline, main bd7da25 (2026-10-28)

`eval/baselines.json` held `null` for every test until this update, so the gates only printed the
measured values. This is the first baseline; there is no earlier baseline to plot as "before".

Values are the CI runner's (GitHub amd64, `docker-full` job), because that is where the gate runs.
ATE comes from the run on main-equivalent commit bd7da25 (PR #29). Latency p95 is the median of the
last three runs (PRs #27, #28, #29); it only warns (shared runner), see `docs/Test_CI_CD.md` PF-01.

| test | ATE RMSE (m) | p95 per-scan latency (ms) | runner p95, PRs #27 / #28 / #29 |
| --- | --- | --- | --- |
| E-01 | 0.002184 | 8.040 | 8.155 / 6.776 / 8.040 |
| E-02 | 0.002184 | 8.079 | 8.164 / 6.830 / 8.079 |
| E-03 | 0.002181 | 8.253 | 8.253 / 7.198 / 8.518 |

Gates with these numbers (`tools/run_integration.py`): E-01 and E-02 fail above 1.2 x 0.002184 =
0.002621 m; E-03 is gated against the E-01 baseline too, its own value is reported. PF-01 warns
above 1.2 x 8.040 = 9.648 ms.

Reproducibility: three local runs on the desktop (WSL, GTSAM 4.2) give byte-identical `est.tum`
for E-01 and E-03 (ATE 0.002183 m for both); the runner's toolchain differs in the last digit.

Configuration at this baseline: `backend.relative_factor: decoupled` (ADR-021), exact pose prior
(#28), gravity-direction factor on keyframe 0 (ADR-020), `degeneracy.enabled: false`.
Data: the synthetic box-circle source (`integration.json` `data_source`), not a Gazebo recording.

Plots (local run, same commit):

- `error_vs_time.png`: horizontal and vertical position error after SE(3) alignment, E-01 and E-03.
  Horizontal stays below 4 mm; vertical peaks at about 12 mm near t = 11 s, when the robot leaves the
  still start and ramps up, then stays below about 5 mm.
- `e01_traj_xy.png`: top view, estimate over ground truth (the 2 mm error is not visible at this scale).

Gazebo recordings (E-01 bridged 0.0955 m, E-04) are not baselined: they are not run in CI
(ADR-011 section 9) and their numbers are tracked in `sim/README.md` until S2.
