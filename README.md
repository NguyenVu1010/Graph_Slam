# multi-sensor-slam

LiDAR-inertial SLAM on a factor graph with GNSS and wheel-odometry factors. C++17 core
without ROS, a thin ROS 2 Jazzy adapter, Gazebo Harmonic simulation, and a latency
benchmark that runs on a Raspberry Pi 5.

Working rules: `CLAUDE.md`. Conventions: `docs/conventions.md`. Decisions: `docs/adr/`.

## Layout

```
core/        C++ library, no ROS (colcon package mss_core)
ros2_node/   ROS 2 adapter (colcon package mss_ros2)
bench/       latency benchmark binary
tests/       unit, component, integration, python
sim/ eval/   Gazebo worlds; evaluation (TUM files only)
docs/        conventions, derivations, adr
docker/      deps image (GTSAM pinned) + multi-stage build
config/      default.yaml, the single source of tunables
```

## Build

Desktop or Pi5, no ROS:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release        # add -DMSS_WITH_GTSAM=OFF without GTSAM
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

WSL (dev machine, Ubuntu 24.04). `setup.sh` is idempotent and needs no sudo: if apt packages
are missing it prints the `sudo apt-get install` line to run, then builds Sophus and GTSAM into
`~/opt`. `build.sh` configures, builds and runs ctest; the last line is `BUILD OK: <dir>`.
Options: `--no-gtsam`, `--asan`, `--build-dir DIR`, `--target T` (builds and tests only `T`).
The default build dir is `~/mss-build/<branch>`.

```sh
tools/wsl/setup.sh
tools/wsl/build.sh --no-gtsam   # then without --no-gtsam for the full build
```

Desktop with ROS 2 Jazzy (workspace contains this repo under `src/`):

```sh
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch mss_ros2 mss.launch.py
```

Docker: see the header of `docker/Dockerfile`.

## Python tooling

```sh
pip install -e ".[dev,eval]"
ruff check . && ruff format .
pytest
```

Simulation noise layer (offline, no ROS; details in `sim/README.md` and `docs/adr/011-simulation.md`):

```sh
PYTHONPATH=sim python -m mss_sim bridge <raw_bag> sim/scenarios/E-01.yaml results/<run-id>   # raw bag -> noisy bag + gt.tum
python tools/mutate_bag.py <in_bag> <out_bag> --drop 0.05 --reorder 0.02                     # E-03 bag: packet loss, reordering
PYTHONPATH=eval python -m mss_eval.noise_report <bag_dir> sim/noise/<profile>.yaml --out results/<run-id>/noise  # declared vs measured noise
```

## Before committing

`pre-commit install` once; it runs clang-format and ruff. CI additionally builds with
`-Werror` and runs the tests.
