#!/usr/bin/env bash
# Launch check: `ros2 launch mss_ros2 mss.launch.py` starts mss_node and it keeps running.
# Passes when the launch is still up after the timeout (timeout exits 124) and the node's
# startup line was printed; any other exit (crash, bad parameters, missing package) fails.
# Usage: check_launch.sh [INSTALL_DIR]   (default /ws/install, the Docker image layout)
# CI:    docker run --rm -v "$PWD:$PWD" -w "$PWD" mss bash tools/ci/check_launch.sh
set -euo pipefail

install_dir="${1:-/ws/install}"
ros_setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
run_s=15    # time the node must stay up, s
kill_s=10   # SIGKILL this long after SIGINT if the launch does not shut down, s
startup_line="mss_node up"

# ROS and colcon setup scripts read unset variables.
set +u
# shellcheck disable=SC1090
. "$ros_setup"
# shellcheck disable=SC1091
. "$install_dir/setup.bash"
set -u

ros2 pkg executables mss_ros2

export RCUTILS_COLORIZED_OUTPUT=0  # plain CI log
log="$(mktemp)"
trap 'rm -f "$log"' EXIT
set +e
timeout --foreground --signal=INT --kill-after="$kill_s" "$run_s" ros2 launch mss_ros2 mss.launch.py 2>&1 | tee "$log"
status="${PIPESTATUS[0]}"
set -e

if [ "$status" -ne 124 ]; then
  echo "LAUNCH CHECK FAILED: ros2 launch exited with $status before ${run_s} s (124 = still running)" >&2
  exit 1
fi
if ! grep -q "$startup_line" "$log"; then
  echo "LAUNCH CHECK FAILED: startup line '$startup_line' not printed" >&2
  exit 1
fi
echo "LAUNCH CHECK OK: mss_node ran for ${run_s} s"
