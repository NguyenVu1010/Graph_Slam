# mcu2d

2D LiDAR + gyro + wheel SLAM for a single-core MCU without an OS (ADR-034). Standalone CMake
project: it does not link `core/`, Eigen, GTSAM or ROS, and does not allocate after start-up.

Milestone M0: ISR ring buffers, message structs, the timestamp merge with the IMU coverage
gate (same rules as `pipeline::Estimator::run_once()/flush()`), and the config struct.

Milestone M1, part A: the scan-to-submap matching core (host only): `Pose2` (SE(2), right
perturbation), the int8 log-odds occupancy submap with Bresenham insertion, an exact uint8
distance field, the correlative + Gauss-Newton scan matcher and the 3x3 degeneracy analysis.
Derivation: `docs/scan_matching.md`. Config: `Config::grid`, `Config::matcher`. Namespace
`mss2d`.

Front end: `MotionModel` (gyro z + wheel or constant velocity, online gyro bias), deskew of
each LiDAR turn into its end frame, and `FrontEnd`, the merger's sink: scan-to-submap odometry
with two overlapping active submaps and a fixed pool of finished ones, degeneracy fallback to
the prediction, one `ScanPose` per scan. `LidarSource` is the driver interface. Config:
`Config::motion`, `Config::submap`, `Config::lidar` (`T_b_l`). Algorithm, synthetic evaluation
(ATE, RPE with and without deskew), host latency and memory: `docs/frontend.md`.

Host simulation (`sim/`, not for the MCU): line-segment worlds, trajectories, an RPLIDAR-like
`SyntheticLidar`, gyro and wheel models, ATE/RPE and TUM output. `mss2d_frontend_eval [--out
DIR]` runs the synthetic loop and, with `--out`, writes `DIR/estimate.tum` and `DIR/truth.tum`.

Host build and test (needs cmake, ninja, libgtest-dev):

```sh
cmake -S mcu2d -B build-mcu2d -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-mcu2d && ctest --test-dir build-mcu2d --output-on-failure
```

ThreadSanitizer: configure with `-DCMAKE_CXX_FLAGS=-fsanitize=thread` and run the test binary
with `setarch $(uname -m) -R` (TSAN needs ASLR off).
