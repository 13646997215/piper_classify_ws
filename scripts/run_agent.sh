#!/usr/bin/env bash
# Natural-language agent for an already running classification simulation.
# Usage:
#   bash scripts/run_agent.sh
#   bash scripts/run_agent.sh --ros-args -p use_voice:=true -p enable_tts:=true
#   After configuring your own key: bash scripts/run_agent.sh --ros-args -p use_llm:=true
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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOG_DIR="${WS_ROOT}/log/agent_runtime"
mkdir -p "${ROS_LOG_DIR}"
exec ros2 run classify_agent agent_node "$@"
