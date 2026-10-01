#!/usr/bin/env bash
# Reload the initial world and task state; no automatic motion by default.
set -eo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "${WS_ROOT}/scripts/stop_classify.sh"
exec bash "${WS_ROOT}/scripts/run_classify.sh" "$@"
