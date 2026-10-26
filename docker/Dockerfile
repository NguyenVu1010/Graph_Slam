# Multi-stage build on top of the deps image.
# docker build -f docker/deps.Dockerfile -t mss-deps .
# docker build -f docker/Dockerfile --build-arg DEPS=mss-deps -t mss .
ARG DEPS=mss-deps
FROM ${DEPS} AS build
WORKDIR /ws/src/multi-sensor-slam
COPY . .
WORKDIR /ws
# colcon also builds the root CMakeLists.txt (superbuild with tests) as package
# multi_sensor_slam; its ctest suite (GTSAM-linked tests included) runs here, and
# colcon test-result exits non-zero on any failure, which fails the image build.
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DMSS_WERROR=ON \
    && colcon test --packages-select multi_sensor_slam --event-handlers console_direct+ \
    && colcon test-result --verbose --all

FROM ${DEPS} AS runtime
COPY --from=build /ws/install /ws/install
CMD ["bash", "-lc", ". /opt/ros/jazzy/setup.sh && . /ws/install/setup.sh && ros2 launch mss_ros2 mss.launch.py"]
