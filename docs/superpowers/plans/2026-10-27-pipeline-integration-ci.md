# Pipeline Integration Tests in CI (E-01..E-03, PF-01) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run E-01, E-02, E-03 and PF-01 on every PR: synthesise the warehouse-lap bag, run the noise bridge and the mssdata converter, replay it through `mss_bench`, and compare ATE and per-scan latency against `eval/baselines.json`.

**Architecture:** One Python harness, `tools/run_integration.py`, chains the existing pieces (`mss_sim.synth` -> `mss_sim.bridge` with `sim/scenarios/E-01.yaml` -> `tools/bag_to_mssdata.py`; `tools/mutate_bag.py` for E-03) and a new `eval/mss_eval/ate.py` (TUM files only, ADR-003). The C++ bench is invoked as an external command, so the harness runs on the GitHub host runner while `mss_bench` runs inside the GTSAM image already built by the `docker-full` CI job. The synthetic generator gains a static start, a smooth speed ramp and the ADR-011 extrinsics so that the IMU can initialise and `config/sim/warehouse.yaml` matches the data.

**Tech Stack:** Python 3.12 (numpy, scipy, pyyaml, rosbags; matplotlib for plots), pytest, ruff; C++17 bench (`bench/main.cpp`); GitHub Actions + Docker.

**Spec:** `docs/Test_CI_CD.md` (`## Test tích hợp và hiệu năng`: E-01..E-03, PF-01; "Workflow, dữ liệu test và quy tắc nhánh": baselines, `integration.yml` row); ADR-009 (regression rule, workflows), ADR-010/014 (TUM, mssdata), ADR-011 (sections 1, 5, 6: 5 s static start, E-03 from E-01 by `mutate_bag.py`, scenario overlay); ADR-003 (`eval/` never links core); `CLAUDE.md` (Tests, Python rules, baseline-update PR); `README.md:40-80`; `sim/README.md`; `docs/derivations/pipeline.md §7` (motion profile, written by the estimator plan). Depends on the estimator plan (`2026-10-27-pipeline-estimator.md`) being merged: before it, `latest_state()` is an identity stub and ATE is meaningless.

## Contract issues

None. No public C++ header changes (`bench.json` gains two keys; it is an output file, not a header). `sim/mss_sim/synth.py` gains keyword arguments whose defaults keep today's output byte for byte.

## Global Constraints

- Python 3.12, type hints on every function, `ruff check . && ruff format --check .` clean (`pyproject.toml`: line length 100, rules E F W I UP B ANN).
- Scripts take paths as arguments and write only under the `--out` directory (`results/<run-id>/` in docs and CI); never into the source tree.
- `eval/` never imports or links core; it reads TUM files only. Quaternions: TUM order `(x, y, z, w)` is owned by `eval/mss_eval/tum.py`.
- Random streams are seeded (scenario `seed: 42`; `mutate_bag.py --seed 42`).
- Regression rule (ADR-009, CLAUDE.md): a PR fails if E-01 ATE RMSE is more than 20 % worse than `eval/baselines.json`; E-03 must stay within the E-01 threshold (Test_CI_CD); E-02 two runs must give identical trajectories; PF-01 only warns at +20 % p95 (shared runner).
- `eval/baselines.json` changes only in a dedicated PR labelled `baseline-update` with before/after plots; this PR leaves it unchanged (all `null`).
- No new dependencies (numpy/scipy/matplotlib/pyyaml/rosbags already in `pyproject.toml`); no evo at run time.
- C++ changes: `-Wall -Wextra -Wpedantic -Werror`, clang-format-18; build via `tools/wsl/build.sh` (`MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd <worktree> && tools/wsl/build.sh --build-dir ~/mss-build/feat-pipeline-ci --target <T>"`; worktree name is the implementer's, adjust the path).
- Python tests run on Windows from the worktree: `python -m pytest tests/python/<file> -v` (WSL has no venv/pip).
- Commits: Conventional Commits (`feat(sim)`, `feat(eval)`, `feat(bench)`, `feat(tools)`, `ci:`), the session's attribution trailers. Do not push.
- PR description: "ATE: first measurement, no baseline yet (baseline-update PR follows)"; latency: no change to core code.

## Review Focus

1. **Est and GT stamps that do not match exactly** (gt.tum written from float seconds loses ns; E-03 drops scans). Expected: nearest-GT association within a tolerance, unmatched estimates ignored and counted; ATE not computed from fewer than a minimum number of pairs. Pinned: Task 2 `test_associate_nearest_within_tolerance`, `test_too_few_pairs_raises`.
2. **`mss_bench` exits 3 on E-03** (records it could not process, e.g. a rejected IMU sample after reordering survives conversion). Expected: E-03 treats 3 as "ran, with warnings", 2 or a crash as failure. Pinned: Task 4 `test_e03_accepts_exit_3`.
3. **First run with `null` baselines.** Expected: pass with a "no baseline" note and the measured numbers printed for the baseline-update PR; never a crash on `None * 1.2`. Pinned: Task 4 `test_null_baseline_passes_with_note`.
4. **E-03 "out of order" is undone by the converter.** `bag_to_mssdata.py` sorts by `header.stamp`, and the mssdata reader rejects backward time, so reordering never reaches `mss_bench`; only drops do. Expected: the harness says so in its report instead of claiming an out-of-order test (the Estimator's out-of-order handling is covered by C-08 and the estimator plan's `OldScanSkipped`). Pinned: Task 4 `test_e03_report_states_reorder_is_resorted`.
5. **Bench run inside Docker with host paths.** Expected: paths given to the bench command are absolute and identical inside and outside the container (workspace mounted at the same path); a missing binary is a setup error (exit 2), not a test failure. Pinned: Task 4 `test_missing_bench_is_setup_error`.

---

## File Structure

| File | Responsibility |
| --- | --- |
| `sim/mss_sim/synth.py` (modify) | `static_s`, `ramp_s`, `t_b_l` keyword arguments (pipeline.md §7) |
| `eval/mss_eval/ate.py` (create) | Association, SE(3) Umeyama alignment, ATE RMSE, xy plot, CLI |
| `bench/main.cpp` (modify) | Per-scan processing latency percentiles in `bench.json` (PF-01) |
| `tools/run_integration.py` (create) | E-01..E-03 + PF-01 harness, baseline comparison, report |
| `.github/workflows/ci.yml` (modify) | Integration steps in the `docker-full` job |
| `tests/python/test_synth.py`, `tests/python/test_ate.py`, `tests/python/test_run_integration.py`, `tests/component/test_bench_replay.cpp` | Tests |
| `README.md`, `eval/README.md` (modify) | How to run the harness locally; baseline policy |

CI decision (dispatch question): E-01 needs the GTSAM backend, so the integration steps go into the existing `docker-full` job, which already builds the deps image with GTSAM 4.2.0 and whose comment reserves it for E-01..E-03. A separate `integration.yml` (ADR-009 naming) would rebuild GTSAM a second time per PR; it is created when `deps-image.yml` publishes the deps image to GHCR (follow-up, noted in the PR).

---

### Task 1: Synthetic generator: static start, speed ramp, ADR extrinsics

**Files:**
- Modify: `sim/mss_sim/synth.py`
- Test: `tests/python/test_synth.py`

**Interfaces:**
- Produces: `write_synthetic_raw_bag(out_dir, *, duration_s=6.0, seed=0, organized=True, box_half_m=(10.0, 6.0, 3.0), omega_radps=0.3, speed_mps=0.5, max_range_m=100.0, origin_lla, static_s: float = 0.0, ramp_s: float = 0.0, t_b_l: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None = None) -> SynthInfo` — `t_b_l = (translation_m, quaternion_wxyz)` Hamilton order; `None` = identity (today's behaviour). Motion per `docs/derivations/pipeline.md §7` with `V = speed_mps`, `W = omega_radps`; `/ground_truth` twist, IMU, joint states (wheel speeds `(v -+ omega * track / 2) / radius`, positions = integrals) and GNSS all from that profile; the LiDAR ray-casts from `T_w_b * T_b_l` and `/tf_static` carries `T_b_l`.

- [x] **Step 1: Write the failing tests**
```python
# Defaults unchanged: the existing tests in test_synth.py (circle, IMU kinematics, tf identity)
# and the bridge/SIM-05 tests keep passing without edits.
def test_static_then_ramp_profile(tmp_path) -> None:
    # static_s=2, ramp_s=2, duration 6, seed 0: for t < 2 s gt position == (0, 0, 0) and
    # twist == 0 exactly; gt position at every stamp equals the pipeline.md §7 closed form (1e-9 m);
    # gt yaw == kappa * s (1e-9 rad); joint-state wheel speeds == (v -+ omega*track/2)/radius (1e-12)
def test_static_then_ramp_imu(tmp_path) -> None:
    # same bag: every IMU sample within 5 sigma (sigma = density * sqrt(200), default profile)
    # of the §7 analytic acc_b / gyro_b
def test_t_b_l_written_and_used(tmp_path) -> None:
    # t_b_l = ((0.10, 0.0, 0.50), (0.99984770, 0.0, 0.01745241, 0.0)): /tf_static carries it;
    # every finite point p_l of the first scan lies on a box plane after T_w_b * T_b_l (1e-6 m)
```
- [x] **Step 2: Run** `python -m pytest tests/python/test_synth.py -v`. Expected: FAIL (unexpected keyword arguments).
- [x] **Step 3: Implement** the profile helper `_profile(t_s, static_s, ramp_s, speed, omega) -> (s, v, a_t)` and use it everywhere `_pose` was used.
- [x] **Step 4: Run** `python -m pytest tests/python -q` (all Python tests, incl. bridge/SIM-05 that use the defaults). Expected: PASS. `python -m ruff check . && python -m ruff format --check .` clean.
- [x] **Step 5: Commit**
```bash
git add sim/mss_sim/synth.py tests/python/test_synth.py
git commit -m "feat(sim): synthetic bag with static start, speed ramp and configurable T_b_l"
```

---

### Task 2: ATE in eval

**Files:**
- Create: `eval/mss_eval/ate.py`, `tests/python/test_ate.py`
- Modify: `eval/README.md`

**Interfaces:**
- Produces (`mss_eval.ate`):
  - `read_tum_stamps_ns(path: Path) -> np.ndarray` — int64 ns parsed from the timestamp text (`sec.frac`, up to 9 fraction digits), no float round trip.
  - `associate(est_ns: np.ndarray, gt_ns: np.ndarray, max_dt_ns: int) -> tuple[np.ndarray, np.ndarray]` — index pairs, nearest GT per estimate, `|dt| <= max_dt_ns`.
  - `umeyama_se3(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]` — R (3x3, det +1), t (3,), minimising `sum |dst - (R src + t)|^2`, no scale.
  - `@dataclass(frozen=True) class AteResult: rmse_m: float; n_pairs: int; n_unmatched: int`
  - `ate(est_path: Path, gt_path: Path, *, max_dt_ns: int = 2_500_000, min_pairs: int = 10) -> AteResult` — `ValueError` when fewer than `min_pairs` pairs. 2.5 ms = half the 200 Hz ground-truth period (planner's call).
  - `plot_xy(est_path: Path, gt_path: Path, out_png: Path) -> None` (aligned est over gt, matplotlib Agg).
  - CLI `python -m mss_eval.ate <est.tum> <gt.tum> [--plot out.png]` prints `{"ate_rmse_m": ..., "n_pairs": ..., "n_unmatched": ...}` as one JSON line; exit 2 on error.

- [x] **Step 1: Write the failing tests**
```python
def test_ev01_self_is_zero(tmp_path) -> None             # EV-01: ate(x, x).rmse_m < 1e-12
def test_rigid_offset_removed(tmp_path) -> None          # est = Rz(30 deg) gt + (5, -3, 1): rmse < 1e-9
def test_known_noise(tmp_path) -> None                   # seed 0, N(0, 0.1^2) per axis on 2000 poses:
                                                          # rmse within 5 % of 0.1 * sqrt(3)
def test_stamps_parsed_exactly(tmp_path) -> None         # "1700000000.123456789" -> 1700000000123456789
def test_associate_nearest_within_tolerance() -> None    # Review Focus 1: est 1 ms off -> matched;
                                                          # 3 ms off -> unmatched, counted
def test_too_few_pairs_raises(tmp_path) -> None          # 9 pairs, min_pairs 10 -> ValueError
def test_cli_prints_json(tmp_path) -> None               # subprocess, exit 0, keys present
```
- [x] **Step 2: Run** `python -m pytest tests/python/test_ate.py -v`. Expected: FAIL (module missing).
- [x] **Step 3: Implement** per Interfaces (Umeyama via SVD of the cross-covariance with the reflection fix).
- [x] **Step 4: Run** the same command, then ruff. Expected: PASS, clean.
- [x] **Step 5: Commit**
```bash
git add eval/mss_eval/ate.py tests/python/test_ate.py eval/README.md
git commit -m "feat(eval): ATE RMSE with SE(3) alignment from TUM files"
```

---

### Task 3: Per-scan latency in bench.json (PF-01)

**Files:**
- Modify: `bench/main.cpp` (Replayer + `write_json` + `print_table`), `tests/component/test_bench_replay.cpp`

**Interfaces:**
- Produces: `bench.json` keys `"scan_process_ms_p50"` and `"scan_process_ms_p95"`: wall time (`std::chrono::steady_clock`) of each `run_once()`/`flush()` call that increased `Stats::n_scans`, divided by the number of scans it processed; nearest-rank percentiles as in `Estimator::stats()`; `0.0` when no scan was processed. Samples kept in a `std::vector<double>` reserved once from the dataset's lidar count (`index.json`), so the replay loop does not allocate (planner's call). Diagnostics only; the estimate is unaffected.

- [x] **Step 1: Write the failing test** — in `ReplaysSyntheticDataset`: `"scan_process_ms_p50"` and `"scan_process_ms_p95"` present, values parse to `>= 0` and `p95 >= p50`.
- [x] **Step 2: Run** `--target test_bench_replay` (GTSAM build command). Expected: FAIL (key missing).
- [x] **Step 3: Implement.**
- [x] **Step 4: Run** the same target, then the no-GTSAM build of the same target. Expected: PASS. Format check (clang-format-18 dry run on the two files) prints nothing.
- [x] **Step 5: Commit**
```bash
git add bench/main.cpp tests/component/test_bench_replay.cpp
git commit -m "feat(bench): per-scan processing latency percentiles in bench.json (PF-01)"
```

---

### Task 4: Integration harness

**Files:**
- Create: `tools/run_integration.py`, `tests/python/test_run_integration.py`

**Interfaces:**
- Consumes: `write_synthetic_raw_bag` (Task 1), `mss_sim.bridge.run_bridge`, `mss_sim.scenario.load_scenario`, `mss_sim.profile.load_profile`, `mss_sim.config_overlay.load_pipeline_config`, `bag_to_mssdata.convert`, `mutate_bag.mutate`, `mss_eval.ate.ate`/`plot_xy` (Task 2), `bench.json` keys (Task 3).
- Produces: CLI
  `python tools/run_integration.py --bench-cmd "<cmd>" --out <dir> [--scenario sim/scenarios/E-01.yaml] [--baselines eval/baselines.json] [--tests E-01,E-02,E-03] [--duration-s S] [--plots] [--path-style native|wsl]`
  and the testable functions
  `prepare(scenario: Path, out: Path, duration_s: float | None) -> Prepared` (`Prepared(mssdata: Path, mssdata_e03: Path, gt_tum: Path, overlay: Path)`),
  `run_bench(bench_cmd: list[str], default_yaml: Path, data: Path, overlay: Path, out: Path, path_style: str) -> int` (exit code; paths made absolute; `wsl` style maps `C:\x\y` to `/mnt/c/x/y`),
  `judge(test_id: str, ate_m: float | None, bench_exit: int, baselines: dict, identical: bool | None) -> Verdict` (`Verdict(passed: bool, note: str)`),
  `main(argv: list[str] | None = None) -> int` — 0 all gated checks pass, 1 a gated check fails, 2 setup error (missing scenario/bench, conversion failure).
- Decisions (planner's call where the sources are silent):
  - `prepare`: synth with `duration_s = scenario.duration_s` (or `--duration-s`), `static_s = 5.0` (ADR-011 §5: every scenario starts with 5 s still), `ramp_s = 2.0`, `box_half_m = (8.0, 5.0, 2.0)` (ceiling and floor inside the ±15° beams), `t_b_l` and `origin_lla` from the scenario's pipeline overlay / scenario; bridge with the scenario seed; `bag_to_mssdata.convert(..., pipeline_config=scenario.pipeline_config)`; E-03: `mutate(bridged, ..., drop_fraction=0.05, reorder_fraction=0.02, window_ns=20_000_000, seed=42)` (ADR-011 §5) then convert.
  - Bench: `<bench-cmd> config/default.yaml <mssdata> --overlay <scenario overlay> --out <out>/<test>/`.
  - E-01: gated on bench exit 0, ATE finite with `>= 10` pairs, and (baseline not null) `ate <= 1.2 * baselines["E-01"]["ate_rmse_m"]`.
  - E-02: second bench run on the E-01 mssdata; gated on `est.tum` byte equality with E-01's.
  - E-03: bench exit in `{0, 3}` (3 = ran with unprocessed records), ATE finite, and (E-01 baseline not null) `ate <= 1.2 * baselines["E-01"]["ate_rmse_m"]` ("ATE trong ngưỡng E-01"); its own `E-03` baseline entry is reported, not gated. Report line: "reordering is re-sorted by bag_to_mssdata.py; only drops reach the bench".
  - PF-01: `scan_process_ms_p95` of E-01 vs `baselines["E-01"]["p95_latency_ms"]`: note "WARN" above 1.2x, never fails.
  - Null baseline: pass with note `"no baseline: record ate_rmse_m=<x> in a baseline-update PR"`.
  - Outputs under `--out`: `E-01/`, `E-02/`, `E-03/` (bench outputs, `ate.json`, optional `traj_xy.png`), `integration.json` (`{test: {ate_rmse_m, n_pairs, bench_exit, baseline, passed, note}}`), `summary.md` (one table, also printed to stdout).

- [x] **Step 1: Write the failing tests** (fast; `--duration-s 8` and a fake bench)
```python
# Fake bench: tests/python write a small Python script into tmp_path that reads the mssdata
# gt.bin (GT_DTYPE from bag_to_mssdata) and writes it as <out>/est.tum plus a bench.json with
# scan_process_ms_p95 = 1.0, then exits with the code given by an env var (default 0).
def test_end_to_end_with_fake_bench(tmp_path) -> None:
    # main([...]) == 0; integration.json has E-01, E-02, E-03 passed, E-01 ate < 1e-6
def test_null_baseline_passes_with_note() -> None         # Review Focus 3: judge("E-01", 0.3, 0, {"E-01": {"ate_rmse_m": None}}, None)
def test_regression_fails() -> None                       # baseline 0.10, ate 0.121 -> not passed; 0.12 -> passed
def test_e02_requires_identical() -> None                 # identical False -> not passed
def test_e03_accepts_exit_3() -> None                     # Review Focus 2: exit 3 passes, exit 2 fails
def test_e03_uses_e01_threshold() -> None                 # E-01 baseline 0.10: E-03 ate 0.125 fails
def test_e03_report_states_reorder_is_resorted(tmp_path)  # Review Focus 4: summary.md contains that line
def test_pf01_only_warns() -> None                        # p95 2x baseline -> passed, note contains "WARN"
def test_missing_bench_is_setup_error(tmp_path) -> None   # Review Focus 5: nonexistent command -> main(...) == 2
def test_wsl_path_style() -> None                         # "C:\\a\\b" -> "/mnt/c/a/b"
def test_writes_only_under_out(tmp_path) -> None          # repo tree unchanged (git status --porcelain empty diff of files touched)
```
- [x] **Step 2: Run** `python -m pytest tests/python/test_run_integration.py -v`. Expected: FAIL (module missing).
- [x] **Step 3: Implement** per Interfaces and Decisions.
- [x] **Step 4: Run** the same command and ruff. Expected: PASS, clean.
- [x] **Step 5: Local run with the real bench** — build `mss_bench` with GTSAM (`--target mss_bench`), then on Windows:
  `python tools/run_integration.py --bench-cmd "wsl -- /home/<user>/mss-build/feat-pipeline-ci/bench/mss_bench" --path-style wsl --out results/local-<date> --plots`
  Expected: exit 0, three rows in `summary.md` with "no baseline" notes, finite ATE values, E-02 identical. Keep the numbers for the PR description; `results/` is not committed.
- [x] **Step 6: Commit**
```bash
git add tools/run_integration.py tests/python/test_run_integration.py
git commit -m "feat(tools): E-01..E-03 and PF-01 integration harness against eval/baselines.json"
```

---

### Task 5: CI wiring and docs

**Files:**
- Modify: `.github/workflows/ci.yml` (`docker-full` job), `README.md` (Python tooling / bench section), `eval/README.md` (baseline policy paragraph)

**Interfaces:**
- Consumes: Task 4 CLI; image `mss` built by the job; installed bench path inside the image.

- [x] **Step 1: Find the installed bench path** — `docker build -f docker/deps.Dockerfile -t mss-deps . && docker build -f docker/Dockerfile --build-arg DEPS=mss-deps -t mss .` (in WSL if Docker is available, else read it from the first CI run's log), then `docker run --rm mss ls /ws/install/multi_sensor_slam/bin`. Expected: `mss_bench` listed (from `install(TARGETS mss_bench RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})`). Use the printed path below.
- [x] **Step 2: Add steps to `docker-full`** after the image build:
```yaml
      - uses: actions/setup-python@v5
        with: { python-version: "3.12" }
      - run: pip install -e ".[eval]"
      - name: Integration E-01..E-03, PF-01
        run: >
          python tools/run_integration.py
          --bench-cmd "docker run --rm -v ${{ github.workspace }}:${{ github.workspace }} -w ${{ github.workspace }} mss /ws/install/multi_sensor_slam/bin/mss_bench"
          --out ${{ github.workspace }}/results/ci-${{ github.run_id }} --plots
      - if: always()
        run: cat results/ci-${{ github.run_id }}/summary.md >> "$GITHUB_STEP_SUMMARY"
      - if: always()
        uses: actions/upload-artifact@v4
        with: { name: integration-${{ github.run_id }}, path: results/ci-${{ github.run_id }} }
```
  Update the job comment: integration runs here because this job already has GTSAM; `integration.yml` follows once `deps-image.yml` publishes the deps image. Results go to the step summary and the artifact (no PR comment: needs a write token on fork PRs; planner's call).
- [x] **Step 3: Docs** — README: an "Integration tests" paragraph with the local command from Task 4 Step 5 and the exit codes; `eval/README.md`: ATE CLI, and the baseline policy: the first CI run prints the numbers; a separate PR labelled `baseline-update` copies `ate_rmse_m` (E-01, E-02, E-03) and `p95_latency_ms` from that run's `integration.json` into `eval/baselines.json`, attaching the `traj_xy.png` plots (before = none on the first update, else the previous artifact).
- [x] **Step 4: Verify** — `python -m pytest tests/python -q` and ruff clean; YAML parses (`python -c "import yaml,sys; yaml.safe_load(open('.github/workflows/ci.yml'))"`). Expected: PASS. The workflow itself is verified by the PR's CI run: `docker-full` green, step summary shows three rows.
- [x] **Step 5: Commit**
```bash
git add .github/workflows/ci.yml README.md eval/README.md
git commit -m "ci: run E-01..E-03 and PF-01 integration in the docker-full job"
```
