#!/usr/bin/env bash
# Stop only the Gazebo launch started by this workspace's run_sim.sh.
# Usage: bash scripts/stop_sim.sh [--quiet]
set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="${WS_ROOT}/log/sim_runtime/launch.pid"
QUIET=false
if [ "${1:-}" = "--quiet" ]; then QUIET=true; fi
say() { if [ "${QUIET}" = false ]; then printf '%s\n' "$*"; fi; }
group_has_live_members() {
  # The launch process may exit before Gazebo finishes shutting down. Check
  # the entire process group; zombies do not keep the simulator alive.
  ps -eo pgid=,stat= | awk -v group="$1" '
    $1 == group && $2 !~ /^Z/ { found=1 }
    END { exit !found }
  '
}

if [ ! -f "${PID_FILE}" ]; then
  say "No simulation launch recorded for this workspace."
  exit 0
fi
read -r launch_pid recorded_start < "${PID_FILE}"
if ! [[ "${launch_pid}" =~ ^[0-9]+$ ]] || ! [[ "${recorded_start}" =~ ^[0-9]+$ ]]; then
  echo "Invalid PID record: ${PID_FILE}; no process was signalled" >&2
  exit 1
fi
if [ ! -r "/proc/${launch_pid}/stat" ]; then
  say "Recorded launch PID ${launch_pid} is gone."
  rm -f "${PID_FILE}"
  exit 0
fi
current_start="$(awk '{print $22}' "/proc/${launch_pid}/stat")"
current_args="$(ps -p "${launch_pid}" -o args= 2>/dev/null || true)"
current_pgid="$(ps -p "${launch_pid}" -o pgid= 2>/dev/null | tr -d ' ' || true)"
if [ "${current_start}" != "${recorded_start}" ] ||
   [[ "${current_args}" != *"ros2 launch classify_description gazebo.launch.py"* ]] ||
   [ "${current_pgid}" != "${launch_pid}" ]; then
  echo "PID record no longer belongs to this workspace's launch; no process was signalled" >&2
  exit 1
fi

say "Stopping this workspace's launch process group ${launch_pid}"
kill -TERM -- "-${launch_pid}"
for attempt in $(seq 1 10); do
  if ! group_has_live_members "${launch_pid}"; then
    rm -f "${PID_FILE}"
    say "Simulation stopped."
    exit 0
  fi
  sleep 1
done

# We validated this exact process and its group before signalling it.
kill -KILL -- "-${launch_pid}" 2>/dev/null || true
rm -f "${PID_FILE}"
say "Simulation stopped after timeout."
