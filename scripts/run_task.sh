#!/usr/bin/env bash
# Run the task server; a separate /classify/start call starts motion.
set -eo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/local_setup.bash"
source "${WS_ROOT}/scripts/tools/runtime_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOG_DIR="${WS_ROOT}/log/task_runtime"
mkdir -p "${ROS_LOG_DIR}"
exec ros2 launch classify_task task.launch.py "$@"
