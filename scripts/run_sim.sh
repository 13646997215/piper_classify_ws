#!/usr/bin/env bash
# Start this workspace's Gazebo launch in its own process group.
# Usage: bash scripts/run_sim.sh [--gui] [--rviz]
set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${WS_ROOT}/log/sim_runtime"
PID_FILE="${RUN_DIR}/launch.pid"
mkdir -p "${RUN_DIR}"

# The user's shell may already have another Piper workspace sourced. Source
# the ROS underlay and *only* our package overlay for this child process.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ ! -f "${WS_ROOT}/install/local_setup.bash" ]; then
  echo "classify_description is not built; run colcon build --symlink-install" >&2
  exit 1
fi
source "${WS_ROOT}/install/local_setup.bash"
source "${WS_ROOT}/scripts/tools/runtime_env.sh"
if [ "$(ros2 pkg prefix classify_description)" != "${WS_ROOT}/install/classify_description" ]; then
  echo "classify_description resolved to a different workspace" >&2
  exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export GAZEBO_MODEL_DATABASE_URI=""

if [ -f "${PID_FILE}" ]; then
  read -r prior_pid prior_start < "${PID_FILE}" || true
  if [ -n "${prior_pid:-}" ] && [ -r "/proc/${prior_pid}/stat" ]; then
    actual_start="$(awk '{print $22}' "/proc/${prior_pid}/stat")"
    args="$(ps -p "${prior_pid}" -o args= 2>/dev/null || true)"
    if [ "${actual_start}" = "${prior_start:-}" ] &&
       [[ "${args}" == *"ros2 launch classify_description gazebo.launch.py"* ]]; then
      echo "Simulation already running as PID ${prior_pid}; stop it with bash scripts/stop_sim.sh" >&2
      exit 1
    fi
  fi
  echo "Removing stale launch PID record" >&2
  rm -f "${PID_FILE}"
fi

GUI=false
RVIZ=false
for arg in "$@"; do
  case "${arg}" in
    --gui) GUI=true ;;
    --rviz) RVIZ=true ;;
    -h|--help)
      sed -n '2,3p' "${BASH_SOURCE[0]}"
      exit 0 ;;
    *) echo "Unknown option: ${arg}" >&2; exit 2 ;;
  esac
done

stamp="$(date +%Y%m%d_%H%M%S)"
LOG="${RUN_DIR}/gazebo_${stamp}.log"
export ROS_LOG_DIR="${RUN_DIR}/ros_${stamp}"
mkdir -p "${ROS_LOG_DIR}"

# setsid gives this launch a process group we own. stop_sim.sh only signals
# this group, rather than killing unrelated gzserver/ROS sessions by name.
setsid ros2 launch classify_description gazebo.launch.py \
  "gui:=${GUI}" "use_rviz:=${RVIZ}" > "${LOG}" 2>&1 < /dev/null &
launch_pid=$!
sleep 1
if ! kill -0 "${launch_pid}" 2>/dev/null; then
  echo "Launch exited immediately; see ${LOG}" >&2
  tail -n 30 "${LOG}" >&2
  exit 1
fi
launch_start="$(awk '{print $22}' "/proc/${launch_pid}/stat")"
printf '%s %s\n' "${launch_pid}" "${launch_start}" > "${PID_FILE}"
printf 'Launched PID %s, ROS_DOMAIN_ID=%s, GUI=%s, RViz=%s\n' \
  "${launch_pid}" "${ROS_DOMAIN_ID}" "${GUI}" "${RVIZ}"
printf 'Log: %s\n' "${LOG}"

# Wait for observable controller activation, not a fixed 45-second sleep.
for attempt in $(seq 1 45); do
  if rg -q '\[ERROR\] \[gzserver-[0-9]+\]: process has died' "${LOG}"; then
    echo "Gazebo server exited before controllers became active; see ${LOG}" >&2
    tail -n 30 "${LOG}" >&2
    bash "${WS_ROOT}/scripts/stop_sim.sh" --quiet
    exit 1
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Launch exited before controllers became active; see ${LOG}" >&2
    tail -n 40 "${LOG}" >&2
    bash "${WS_ROOT}/scripts/stop_sim.sh" --quiet
    exit 1
  fi
  controllers="$(timeout 5 python3 "${WS_ROOT}/scripts/tools/check_controllers.py" 2>/dev/null || true)"
  if [[ "${controllers}" == *"piper_joint_state_broadcaster"* ]] &&
     [[ "${controllers}" == *"piper_arm_controller"* ]] &&
     [[ "${controllers}" == *"piper_gripper_controller"* ]] &&
     [ "$(printf '%s\n' "${controllers}" | rg -c 'active')" -ge 3 ]; then
    printf '%s\n' "${controllers}"
    echo "Simulation ready. Stop with bash scripts/stop_sim.sh"
    exit 0
  fi
  sleep 2
done
echo "Controllers did not all become active; see ${LOG}" >&2
tail -n 40 "${LOG}" >&2
bash "${WS_ROOT}/scripts/stop_sim.sh" --quiet
exit 1
