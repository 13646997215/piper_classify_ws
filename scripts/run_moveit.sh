#!/usr/bin/env bash
# Start MoveIt for a running classify_description Gazebo simulation.
# Usage: bash scripts/run_moveit.sh [--rviz]
# This script runs in the foreground; press Ctrl+C to stop MoveIt.
set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RVIZ=false
case "${1:-}" in
  --rviz) RVIZ=true ;;
  -h|--help) sed -n '2,3p' "${BASH_SOURCE[0]}"; exit 0 ;;
  '') ;;
  *) echo "Usage: bash scripts/run_moveit.sh [--rviz]" >&2; exit 2 ;;
esac
if [ "$#" -gt 1 ]; then
  echo "Usage: bash scripts/run_moveit.sh [--rviz]" >&2
  exit 2
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ ! -f "${WS_ROOT}/install/local_setup.bash" ]; then
  echo "Workspace is not built; run colcon build --symlink-install" >&2
  exit 1
fi
source "${WS_ROOT}/install/local_setup.bash"
if [ "$(ros2 pkg prefix classify_moveit_config)" != "${WS_ROOT}/install/classify_moveit_config" ]; then
  echo "classify_moveit_config resolved outside this workspace" >&2
  exit 1
fi
if [ "$(ros2 pkg prefix classify_scene_bridge 2>/dev/null || true)" != "${WS_ROOT}/install/classify_scene_bridge" ]; then
  echo "classify_scene_bridge is not built in this workspace; run colcon build --packages-up-to classify_moveit_config --symlink-install" >&2
  exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOG_DIR="${WS_ROOT}/log/moveit_runtime"
mkdir -p "${ROS_LOG_DIR}"
exec ros2 launch classify_moveit_config move_group.launch.py "use_rviz:=${RVIZ}"
