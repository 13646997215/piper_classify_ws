#!/usr/bin/env bash
# Run C++ RGB-D detection against the classify Gazebo simulation.
set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ ! -f "${WS_ROOT}/install/local_setup.bash" ]; then
  echo "Workspace is not built; run colcon build --symlink-install" >&2
  exit 1
fi
source "${WS_ROOT}/install/local_setup.bash"
source "${WS_ROOT}/scripts/tools/runtime_env.sh"
if [ "$(ros2 pkg prefix classify_perception)" != "${WS_ROOT}/install/classify_perception" ]; then
  echo "classify_perception resolved outside this workspace" >&2
  exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOG_DIR="${WS_ROOT}/log/perception_runtime"
mkdir -p "${ROS_LOG_DIR}"
exec ros2 launch classify_perception perception.launch.py "$@"
