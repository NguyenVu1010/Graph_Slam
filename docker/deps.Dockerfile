# Dependency image: ROS 2 Jazzy + pinned GTSAM. Rebuild only when a pin changes.
FROM ros:jazzy-ros-base-noble

ARG GTSAM_VERSION=4.2.0
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git \
      libeigen3-dev libboost-all-dev libtbb-dev libgtest-dev libyaml-cpp-dev \
      clang-format-18 clang-tidy-18 python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${GTSAM_VERSION} https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build -G Ninja \
         -DCMAKE_BUILD_TYPE=Release \
         -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    && cmake --build /tmp/gtsam/build --parallel 2 \
    && cmake --install /tmp/gtsam/build \
    && rm -rf /tmp/gtsam

RUN pip3 install --break-system-packages ruff pytest numpy
