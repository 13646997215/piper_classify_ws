#!/usr/bin/env bash
# Stop owned launch groups, including Gazebo left after the launcher exits.
# Usage: bash scripts/stop_classify.sh [--quiet] [--expected-pid PID]
set -eo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "${WS_ROOT}/scripts/tools/classify_lifecycle.py" "$@"
