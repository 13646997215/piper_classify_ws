#!/usr/bin/env bash
# Verify that MoveIt plans a small joint1 motion. Add --execute to move and
# return the arm, after Gazebo and move_group are running.
set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXECUTE=false
if [ "${1:-}" = "--execute" ]; then
  EXECUTE=true
elif [ "$#" -ne 0 ]; then
  echo "Usage: bash scripts/verify_moveit.sh [--execute]" >&2
  exit 2
fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/local_setup.bash"
source "${WS_ROOT}/scripts/tools/runtime_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOG_DIR="${WS_ROOT}/log/moveit_runtime"
mkdir -p "${ROS_LOG_DIR}"
exec ros2 launch classify_moveit_config verify_joint_motion.launch.py "execute:=${EXECUTE}"
