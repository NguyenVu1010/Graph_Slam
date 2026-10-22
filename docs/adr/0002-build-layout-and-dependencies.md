# ADR-0002: Build layout and initial dependencies

- Status: accepted
- Date: 2026-10-22

## Context

`core/` must build on a Pi5 without ROS and be consumed by a ROS 2 Jazzy node on the
desktop. GTSAM is heavy and must not leak into modules other than `backend`.

## Decision

- Two colcon packages: `core/` (`mss_core`, plain CMake, exports `mss::*` targets) and
  `ros2_node/` (`mss_ros2`, ament_cmake, depends on `mss_core`).
- A root `CMakeLists.txt` is a superbuild for non-ROS use: core + `tests/` + `bench/`.
- Each core module is its own CMake target `mss::<module>`. Modules without sources are
  `INTERFACE` targets so the dependency direction (pipeline -> modules -> common) is
  enforced by the linker from day one. `apps` (bench, ros2_node) link only `mss::pipeline`.
- `MSS_WITH_GTSAM` (default ON) links GTSAM into `mss::backend` only. OFF builds the rest
  of core without GTSAM so unit tests and lint can run on a thin CI runner.
- Shared compile flags live in `cmake/mss_warnings.cmake`: `-Wall -Wextra -Wpedantic`,
  `-Werror` via `MSS_WERROR` (CI turns it on).
- Config is one file, `config/default.yaml`, installed into the ROS package share and read
  directly by bench. `deterministic: true` is the default.
- Dependencies: Eigen3 >= 3.4, GTSAM (pinned in `docker/deps.Dockerfile`), GTest.
  yaml-cpp will be added to `pipeline` when config loading lands, under this ADR.
- Python tooling is one `pyproject.toml` at the root (`ruff`, `pytest`), packages under
  `eval/`, `sim/`, `tools/`. `eval` never imports core bindings; it reads TUM files.

## Consequences

- Adding a module source: change its `CMakeLists.txt` from `INTERFACE` to `STATIC`.
- Pi5 builds use the root CMake with `-DMSS_BUILD_TESTS=OFF` to save RAM.
- Any new third-party dependency amends this ADR or gets its own.
