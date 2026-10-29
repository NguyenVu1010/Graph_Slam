# Baseline update G13: `gnss.align_check_yaw_sigma` on at 6 deg (2026-10-29)

Owner decision (backlog G13, ADR-025): the GnssAligner yaw-sigma check is on by default, with the
6 deg threshold `gnss.align_yaw_max_sigma_deg` that designs 2 and 2b share. E-01..E-03 output
changes (a different alignment time and 214 GNSS factors instead of 223 on E-01), so the
baselines change in this dedicated `baseline-update` PR. Before = main e208971 (flag off); after =
this branch (flag on). The previous baseline is `docs/baselines/2026-10-28/`.

## Before and after

| test | ATE before (m) | ATE after (m) | limit before (1.2 x) | limit after (1.2 x) | p95 latency baseline (ms) |
| --- | --- | --- | --- | --- | --- |
| E-01 | 0.002184 | 0.002160 | 0.002621 | 0.002592 | 8.040 (unchanged) |
| E-02 | 0.002184 | 0.002160 | 0.002621 | 0.002592 | 8.079 (unchanged) |
| E-03 | 0.002181 | 0.002172 | 0.002621 (gated on E-01) | 0.002592 (gated on E-01) | 8.253 (unchanged) |

E-02 is the second run on the E-01 dataset (byte-identical `est.tum`), so it shares E-01's ATE.
E-03 is gated against the E-01 baseline; its own value is reported.

The "after" values are this machine's (WSL, GTSAM 4.2). The runner's toolchain differs in the last
digit, so the values in `eval/baselines.json` are replaced by the CI run of this PR when it differs
(as in the first baseline PR). Latency baselines are unchanged: local p95 stays at 6.1 to 6.5 ms
before and after, no measurable change (the check adds three running sums per fix, off the scan path).

## Same data, same seeds

Three local runs each, before and after: `est.tum` byte-identical across the three runs of each
state. md5 before E-01 and E-02 `d1b1e4ba...`, E-03 `ca02a49f...` (the same as main in ADR-025);
after E-01 and E-02 `a2eef2c1...`, E-03 `ee007064...`.

Local ATE (m): before 0.002183 / 0.002183 / 0.002183 (E-01 / E-02 / E-03), after
0.002160 / 0.002160 / 0.002172.

## What changed in the run

The aligner waits until its predicted yaw sigma is at most 6 deg: E-01 aligns at 14.8 s after
4.4 m (before: 13 s, 3.5 m), the yaw 15.4 deg off with sigma 5.97 deg (2.6 sigma, within 3 sigma;
before 21 deg off at sigma 7.6 deg). The first alignment is still not good; the graph refines E
afterwards. Study: `docs/derivations/gnss_fusion_study.md` section 17. The 15-seed corridor and
15-seed box-world studies are unchanged on every seed.

## Plots (local run)

- `error_vs_time.png`: horizontal and vertical position error after SE(3) alignment, E-01 and E-03,
  before and after. The two curves overlap: the ATE change is 1 % (E-01) and 0.5 % (E-03). The
  maxima are 3.67 / 3.68 mm horizontal and 11.63 / 11.66 mm vertical on E-01 (before / after), with
  the same vertical peak near t = 11 s as in the first baseline.
- `e01_traj_xy.png`: E-01 top view (estimate over ground truth, not distinguishable at this scale)
  and the horizontal error vector est - gt in mm for both states.

Configuration: `config/default.yaml` + `config/sim/warehouse.yaml` (E-01 overlay, which does not set
the flag, so it takes the default). Data: the synthetic box circle of `integration.json`
`data_source`, not a Gazebo recording.

## Reverting

`gnss.align_check_yaw_sigma: false` in the YAML restores the previous behaviour. The struct default
`GnssConfig::align_check_yaw_sigma` is `true` too, so code that builds a `GnssConfig` without YAML
gets the check.
