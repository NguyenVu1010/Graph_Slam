# eval

Reads TUM-format trajectories only; never links against core.
`baselines.json` changes only in a PR labelled baseline-update (see Baselines below).

## Noise report

Compares the declared noise profile with the noise measured in a bridged sim bag (ADR-011
section 7):

```
PYTHONPATH=eval python -m mss_eval.noise_report <bag_dir> sim/noise/<profile>.yaml --out <dir>
    [--planes planes.yaml] [--pipeline-config cfg.yaml] [--static-s S] [--allan-gate] [--odom-gate]
```

`<bag_dir>` is the bridge output dir (with `bag/`) or the bag itself. It writes
`noise_report.html` and `noise_report.json` and checks five sections: IMU (Allan deviation vs
declared densities, from an initial static window of `--static-s` seconds), GNSS (sigma, outage and
multipath), LiDAR (along-ray range sigma, dropout), odom (drift) and Bag (topics, ordering,
ground truth present). Rows are either gated or informational. The IMU Allan rows gate only
with `--allan-gate` and the odom drift (1-3 %) only with `--odom-gate`; other rows are
informational unless noted in the report. Exit code is 0 when every gated row passes and 1 when
one fails, so a bag with the wrong noise is never packaged.

LiDAR sigma needs known planes: pass `--planes` (YAML list of `{n: [x, y, z], d: m}`, with
`n . p = d`); without it the LiDAR sigma row is skipped. The fixture synthetic bag has no LiDAR
range noise, so `--planes` on it reports a sigma mismatch (known limitation). Bags from
`tools/mutate_bag.py` (E-03) are not run through the report. The physics row and the skew vs
no-skew figure of ADR section 7 arrive with the Gazebo plan.

## ATE

Absolute trajectory error of an estimate against ground truth, both TUM files (`mss_bench`
writes `est.tum`, the bridge writes `gt.tum`):

```
PYTHONPATH=eval python -m mss_eval.ate <est.tum> <gt.tum> [--plot traj_xy.png]
    [--max-dt-ns 2500000] [--min-pairs 10]
```

It prints `{"ate_rmse_m": ..., "n_pairs": ..., "n_unmatched": ...}` as one JSON line (exit 2
on error). Stamps are parsed from the text into integer nanoseconds. Each estimate is paired
with the nearest ground-truth stamp within `--max-dt-ns` (default 2.5 ms, half the 200 Hz
ground-truth period); estimates without one are counted in `n_unmatched` and ignored, and
fewer than `--min-pairs` pairs is an error. The estimate is aligned to the ground truth with
a rigid SE(3) Umeyama fit (no scale) before the RMSE of the position error is taken.
`--plot` writes the aligned top view. No evo is needed at run time.

## Map picture

Top-down picture of the global map that `mss_bench --map-out` writes, with the trajectories:

```
PYTHONPATH=eval python -m mss_eval map-png <map.pcd> [--est est.tum] [--gt gt.tum] --out map.png
    [--z-min-m 0.3] [--z-max-m 2.0] [--res-m 0.05] [--width-px 1600] [--title E-01]
```

It reads PCD v0.7 (`DATA binary` or `ascii`, fields `x y z`) and TUM files only. Points with
`z` between `--z-min-m` and `--z-max-m` (frame `w`, m; the defaults drop the floor and the
ceiling of the sim worlds) are counted per `--res-m` cell and drawn as log(1 + count), dark =
many points. The estimate is drawn as written, in `w`. The ground truth is brought into `w`
with the inverse of the SE(3) fit that `mss_eval.ate` uses, so the map is never moved; with both
files the title carries the ATE RMSE. The PNG is `--width-px` wide and as tall as the map's
aspect needs (480 to 4000 px). It prints one JSON line (`n_points`, `n_in_cut`, grid size, image
size, `ate_rmse_m`, `n_pairs`); a missing, truncated or non-PCD file, a bad TUM file or
`--z-min-m >= --z-max-m` exits 2 with `map-png: error: ...` on stderr. `tools/run_map_demo.sh`
(sim/README.md "Map demo") runs it after a recording.

## Baselines

`baselines.json` holds, per integration test (E-01, E-02, E-03), the reference `ate_rmse_m`
and `p95_latency_ms` that `tools/run_integration.py` compares against: E-01 (and E-03, which
uses the E-01 threshold) fails when ATE RMSE is more than 20 % above the E-01 value; the p95
per-scan latency of E-01 (PF-01) only warns. While a value is `null` the check passes with a
"no baseline" note that prints the measured number. The first CI run therefore only reports;
a separate PR labelled `baseline-update` then copies `ate_rmse_m` (E-01, E-02, E-03) and
`p95_latency_ms` from that run's `integration.json` (artifact `integration-<run-id>`) into
`baselines.json` and attaches the `traj_xy.png` plots, before and after (before = none on the
first update, else the previous baseline run's artifact). No other PR changes this file.
