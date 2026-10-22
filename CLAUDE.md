# multi-sensor-slam — working rules

LiDAR-inertial SLAM on a factor graph, with GNSS and wheel-odometry factors,
written in C++17. It runs on a desktop (Gazebo Harmonic, ROS 2 Jazzy, Ubuntu 24.04)
and in real time on a Raspberry Pi 5 with a LiDAR and an MPU IMU that an FPGA reads.

These rules apply to every change, whether a person or an AI assistant writes it.
Architecture decisions live in `docs/adr/`. When a change goes against an ADR,
write a new ADR first.

## Repository layout

```
core/        C++ library, no ROS: common, preprocess, map, scan_match, degeneracy,
             imu, gnss, odom, loop, backend (only place that includes GTSAM), pipeline
ros2_node/   thin ROS 2 adapter: message <-> core struct, launch, params
bench/       latency benchmark binary (runs on Pi5 without ROS)
sim/         Gazebo worlds (SDF), trajectories, sensor noise models
eval/        Python: evo, ablation, plots, baselines.json
docs/        conventions.md, derivations, adr/
docker/      multi-stage Dockerfile, deps image (GTSAM pinned)
tests/       unit, component, integration (bags fetched by tools/fetch_test_data.sh)
```

Dependency direction is fixed: apps -> pipeline -> modules -> common.
- `core/` never includes ROS headers.
- Only `core/backend` includes GTSAM. Other modules return core types (pose, covariance).
- `eval/` never links against core. It only reads TUM-format trajectory files.

## Math and frame conventions (read docs/conventions.md)

- Transform names carry their frames: `T_w_b` maps points from frame `b` to frame `w`,
  so `p_w = T_w_b * p_b`. Frames: `w` world/map, `b` body = IMU, `l` LiDAR, `e` ENU,
  `g` GNSS antenna. Composition must cancel: `T_w_l = T_w_b * T_b_l`.
- Rotations `R_a_b`, positions `p_a_b` (origin of `b` expressed in `a`), velocities `v_w_b`.
- One perturbation convention: right perturbation on SE(3), `T <- T * Exp(delta)`,
  tangent order `[rho, phi]` (translation, rotation), the same as Sophus; GTSAM `Pose3` is `[phi, rho]` and only `backend` permutes.
- Jacobian names: `J_<residual>_<variable>`, for example `J_r_xi`, `J_dp_bg`.
- Units are SI. Angles in code are radians. Degrees appear only in YAML and logs, and
  the key is suffixed `_deg`.
- Quaternions are only used at I/O boundaries, in Hamilton order (w, x, y, z). Write the
  order in the code comment wherever one is constructed.
- Time is `Timestamp` (int64 nanoseconds, sensor clock). Convert to double seconds only
  to compute `dt`. Never compare floating-point timestamps.
- Gravity is `[0, 0, -9.80665]` in `w` unless the IMU initialisation estimates it.

## C++ rules

- C++17. GCC 13 and Clang. Build warnings: `-Wall -Wextra -Wpedantic`; CI adds `-Werror`.
- Formatting comes from `.clang-format` and static checks from `.clang-tidy`. Run both
  before committing (pre-commit does this).
- Naming: types `PascalCase`; functions, methods and variables `snake_case`; members get a
  trailing underscore (`map_`); constants `kPascalCase`; namespaces `mss::<module>`.
- Headers use `#pragma once`. Includes go in this order: own header, C/C++ std, third
  party, project.
- Do not use `auto` for Eigen expressions. Assign them to a concrete type, such as
  `Eigen::Matrix<double, 6, 1>`, so they are evaluated.
- Prefer fixed-size Eigen types in hot code. Pass Eigen objects by `const&`.
- No magic numbers. Every tunable value lives in a config struct loaded from YAML, has a
  unit in its name or comment, and is validated at start-up with a clear error message.
- Errors:
  - Initialisation and config may throw.
  - Per-message processing returns a status (`Result<T>` / `std::optional` plus a reason).
  - Never `abort` on bad sensor data; log and skip.
- Ownership: prefer values and `std::unique_ptr`. Use `std::shared_ptr` only for data
  that threads truly share, and document which ones.
- Threads are created only by `pipeline::Estimator`. Modules are single-threaded and
  talk through bounded queues. Any shared state has one documented owner and a mutex.
- Logging uses the core logger interface. `ros2_node` adapts it to rclcpp logging.
  Do not use `std::cout` in core.

## Hot path rules (code that runs per scan or per IMU sample)

- No heap allocation, no logging and no exceptions per point. Reserve buffers once and
  reuse them.
- Do not copy point clouds. Pass spans or references.
- Every change to the hot path reports `bench` numbers before and after (p50, p95, CPU)
  on the desktop. Changes to scan matching also report Pi5 numbers.
- Keep a single-thread, fixed-seed mode (`deterministic: true` in YAML) so runs can be
  reproduced bit for bit.

## Tests (see docs and the Test & CI plan)

- Every analytic Jacobian has a test against numerical differentiation (relative error
  < 1e-6). GTSAM factors also pass `EXPECT_CORRECT_FACTOR_JACOBIANS`.
- Every module has unit tests, plus component tests on synthetic data with a known answer.
- Random tests use fixed seeds.
- Integration tests E-01 to E-03 run on every PR and compare against
  `eval/baselines.json`. A PR fails if ATE gets more than 20% worse.
- Only change `eval/baselines.json` in a dedicated PR labelled `baseline-update`, with
  before and after plots.

## Python (sim/, eval/, tools/)

- Python 3.12, type hints, `ruff` for lint and format, `pytest`.
- Scripts take paths as arguments and write results to `results/<run-id>/`. They never
  write into the source tree.

## Git and PRs

- Trunk-based. `main` is protected. Squash merge.
- Use Conventional Commits (`feat(scan_match): ...`, `fix(imu): ...`, `perf:`, `test:`,
  `docs:`).
- Keep PRs small: aim for 400 changed lines or fewer.
- The PR description states what changed, which tests cover it, and the effect on ATE and
  latency, with numbers or "no change expected".
- No FPT or other employer code, data or hardware is used in this repo.

## Definition of done for a feature

1. The derivation is in `docs/`, and an ADR exists if a design choice was made.
2. Jacobian and unit tests pass. A component test with a known answer passes.
3. The feature is behind a YAML flag that defaults to the previous behaviour until it is
   evaluated.
4. Integration tests pass, and the evaluation report is attached to the PR.
5. For hot-path changes, `bench` numbers are attached.
6. README or docs are updated if behaviour or parameters changed.

## Notes for AI assistants

- Read the relevant ADR and `docs/conventions.md` before editing a module.
- Before writing a Jacobian, state the residual, the variables and the perturbation
  convention in a comment, then write the numerical-difference test.
- Do not add dependencies, change the build system or change frame conventions without
  an ADR.
- When a task is ambiguous about frames or units, ask instead of guessing.
- Build and test C++ only through `tools/wsl/build.sh [--no-gtsam] [--target T]` from WSL
  (Windows has no toolchain).
- Interface headers under `core/<module>/include/mss/<module>/*.hpp` are the contract.
  Changing a signature needs the controller's or owner's agreement and a note in the PR.
- Run `colcon build --parallel-workers 2` (the dev machine has 16 GB RAM) and the
  affected tests before saying a change is finished.
