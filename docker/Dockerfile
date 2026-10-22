# Multi-stage build on top of the deps image.
# docker build -f docker/deps.Dockerfile -t mss-deps .
# docker build -f docker/Dockerfile --build-arg DEPS=mss-deps -t mss .
ARG DEPS=mss-deps
FROM ${DEPS} AS build
WORKDIR /ws/src/multi-sensor-slam
COPY . .
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DMSS_WERROR=ON

FROM ${DEPS} AS runtime
COPY --from=build /ws/install /ws/install
CMD ["bash", "-lc", ". /opt/ros/jazzy/setup.sh && . /ws/install/setup.sh && ros2 launch mss_ros2 mss.launch.py"]
