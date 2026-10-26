# eval

Reads TUM-format trajectories only; never links against core.
`baselines.json` changes only in a PR labelled baseline-update.

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
