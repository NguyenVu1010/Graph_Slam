# mcu2d

2D LiDAR + gyro + wheel SLAM for a single-core MCU without an OS (ADR-034). Standalone CMake
project: it does not link `core/`, Eigen, GTSAM or ROS, and does not allocate after start-up.

Milestone M0 (this slice): ISR ring buffers, message structs, the timestamp merge with the
IMU coverage gate (same rules as `pipeline::Estimator::run_once()/flush()`), and the config
struct. No SLAM algorithm yet. Namespace `mss2d`.

Host build and test (needs cmake, ninja, libgtest-dev):

```sh
cmake -S mcu2d -B build-mcu2d -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-mcu2d && ctest --test-dir build-mcu2d --output-on-failure
```

ThreadSanitizer: configure with `-DCMAKE_CXX_FLAGS=-fsanitize=thread` and run the test binary
with `setarch $(uname -m) -R` (TSAN needs ASLR off).
