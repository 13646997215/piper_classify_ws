#!/usr/bin/env bash
# Complete simulation; Ctrl+C stops this launch and its children.
# Usage: bash scripts/run_classify.sh [--headless] [--auto-start] [--plan-only] [--world PATH] [--sensor-stress] [--seed N]
set -eo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_LOG_DIR="${WS_ROOT}/log/classify_runtime"
mkdir -p "${ROS_LOG_DIR}"
export CLASSIFY_WS_ROOT="${WS_ROOT}"
# Keep PID creation atomic with stop and with another launch wrapper.
exec {classify_lock_fd}>"${ROS_LOG_DIR}/lifecycle.lock"
flock -x "${classify_lock_fd}"
PID_FILE="${ROS_LOG_DIR}/launch.pid"
if ! python3 "${WS_ROOT}/scripts/tools/classify_lifecycle.py" --check; then
  echo 'This workspace still has a launch or orphaned simulator; run bash scripts/stop_classify.sh first.' >&2
  exit 1
fi
if [ -f "${PID_FILE}" ]; then
  read -r prior_pid prior_start < "${PID_FILE}" || true
  if [ -n "${prior_pid:-}" ] && [ -r "/proc/${prior_pid}/stat" ] &&
      [ "$(awk '{print $22}' "/proc/${prior_pid}/stat")" = "${prior_start:-}" ]; then
    echo 'This workspace already has a complete launch; use bash scripts/stop_classify.sh first.' >&2
    exit 1
  fi
  rm -f "${PID_FILE}"
fi
GUI=true; RVIZ=true; AUTO=false; PLAN=false
WORLD="${WS_ROOT}/src/description/classify_description/worlds/classify.world"
NOISE=0.0; DROPOUT=0.0; SEED=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --headless) GUI=false; RVIZ=false ;;
    --auto-start) AUTO=true ;;
    --plan-only) PLAN=true ;;
    --world) [ "$#" -ge 2 ] && [ -f "$2" ] || { echo 'Expected --world existing-file' >&2; exit 2; }; WORLD="$(realpath "$2")"; shift ;;
    --sensor-stress) NOISE=0.001; DROPOUT=0.05 ;;
    --seed) [ "$#" -ge 2 ] && [[ "$2" =~ ^[0-9]+$ ]] || { echo 'Expected --seed nonnegative-integer' >&2; exit 2; }; SEED="$2"; shift ;;
    -h|--help) sed -n '2,3p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
if [ "$(ros2 pkg prefix classify_bringup)" != "${WS_ROOT}/install/classify_bringup" ]; then
  echo 'Build this workspace first: colcon build --symlink-install' >&2; exit 1
fi
# Read-only preflight prevents two simulations in the same DDS domain.
if python3 "${WS_ROOT}/scripts/tools/check_controllers.py" >/dev/null 2>&1; then
  echo 'A controller manager already runs in this ROS_DOMAIN_ID; stop the old simulation first.' >&2; exit 1
fi
setsid ros2 launch classify_bringup classify.launch.py "gui:=${GUI}" "rviz:=${RVIZ}" "auto_start:=${AUTO}" "plan_only:=${PLAN}" \
  "world:=${WORLD}" "depth_noise_stddev_m:=${NOISE}" "depth_dropout_probability:=${DROPOUT}" "stress_seed:=${SEED}" < /dev/null {classify_lock_fd}>&- &
classify_pid=$!
sleep 1
if ! kill -0 "$classify_pid" 2>/dev/null; then
  wait "$classify_pid"
  exit 1
fi
printf '%s %s\n' "$classify_pid" "$(awk '{print $22}' "/proc/${classify_pid}/stat")" > "${PID_FILE}"
flock -u "${classify_lock_fd}"
exec {classify_lock_fd}>&-
trap 'bash "${WS_ROOT}/scripts/stop_classify.sh" --quiet --expected-pid "$classify_pid"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait "$classify_pid"
