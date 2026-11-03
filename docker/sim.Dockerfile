# Desktop simulation image (ADR-011 section 9, optional, never built in CI): ROS 2 Jazzy +
# Gazebo Harmonic (through ros-gz) + the Python deps of sim/ and eval/. It does not build core/.
# rviz2, joy, teleop_twist_joy, teleop_twist_keyboard: run dependencies of mss_teleop
# (ADR-031); visualization_msgs, std_srvs and rosgraph_msgs come with ros-base (ros_core).
#
#   docker build -f docker/sim.Dockerfile -t mss-sim .
#   docker run --rm --gpus all -v "$PWD:$PWD" -w "$PWD" mss-sim \
#     tools/record_scenario.sh sim/scenarios/E-01.yaml results/<run-id>/
#
# Without a GPU add --render-engine ogre (slower). Results land in the mounted results/.
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-tf2-ros \
      ros-jazzy-rviz2 ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard \
      python3-pip python3-numpy python3-scipy python3-yaml \
      python3-jinja2 python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# Pure-Python deps not packaged for Ubuntu 24.04 (versions as in pyproject.toml).
RUN pip3 install --no-cache-dir --break-system-packages \
      "rosbags>=0.11" "shapely>=2.0" "allantools>=2024.6"

ENV PYTHON=python3
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec \"$@\"", "--"]
