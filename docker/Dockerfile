# Multi-stage build on top of the deps image.
# docker build -f docker/deps.Dockerfile -t mss-deps .
# docker build -f docker/Dockerfile --build-arg DEPS=mss-deps -t mss .
ARG DEPS=mss-deps
FROM ${DEPS} AS build
WORKDIR /ws/src/multi-sensor-slam
COPY . .
WORKDIR /ws
# Three colcon packages (ADR-013): multi_sensor_slam (root superbuild: core + tests + bench,
# bench at /ws/install/multi_sensor_slam/bin/mss_bench), mss_core (core/) and mss_ros2
# (ros2_node/, finds mss_core through colcon's prefix path). colcon stops descending at the
# root CMakeLists.txt, so every package path is listed with --paths; colcon list prints them.
# One package at a time (--parallel-workers 1), each with colcon's default make -j<nproc>.
# The superbuild's ctest suite (GTSAM-linked tests included) runs here, and colcon
# test-result exits non-zero on any failure, which fails the image build.
RUN . /opt/ros/jazzy/setup.sh \
    && pkgs="src/multi-sensor-slam src/multi-sensor-slam/core src/multi-sensor-slam/ros2_node" \
    && colcon list --paths $pkgs \
    && colcon build --paths $pkgs --parallel-workers 1 \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DMSS_WERROR=ON \
    && colcon test --paths $pkgs --packages-select multi_sensor_slam --event-handlers console_direct+ \
    && colcon test-result --verbose --all

FROM ${DEPS} AS runtime
COPY --from=build /ws/install /ws/install
CMD ["bash", "-lc", ". /opt/ros/jazzy/setup.sh && . /ws/install/setup.sh && ros2 launch mss_ros2 mss.launch.py"]
