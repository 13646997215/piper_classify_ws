#!/usr/bin/env bash
# Source after the ROS underlay/overlay; only adjusts this process environment.
# Explicit DDS profiles/RMW and an explicit display are respected.
classify_runtime_env() {
  local classify_root classify_profile classify_display classify_auth classify_render_ok
  classify_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  classify_profile="${classify_root}/scripts/config/fastdds_local.xml"
  if [ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" = "rmw_fastrtps_cpp" ] &&
     [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}${FASTDDS_DEFAULT_PROFILES_FILE:-}${FASTDDS_BUILTIN_TRANSPORTS:-}" ] &&
     [ "${CLASSIFY_DEFAULT_DDS:-1}" != "0" ]; then
    if [ -r "$classify_profile" ]; then
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export FASTRTPS_DEFAULT_PROFILES_FILE="$classify_profile"
      printf '[classify env] Fast DDS RGB-D profile: %s (32 MiB per participant)\n' "$classify_profile" >&2
    fi
  fi
  if [ "${CLASSIFY_AUTO_DISPLAY:-1}" != "0" ] &&
     { [ -z "${DISPLAY:-}" ] || [ -z "${XAUTHORITY:-}" ]; }; then
    while IFS= read -r -d '' classify_display && IFS= read -r -d '' classify_auth; do
      [ -z "${DISPLAY:-}" ] || [ "$DISPLAY" = "$classify_display" ] || continue
      classify_render_ok=1
      if command -v glxinfo >/dev/null; then
        if [ -n "$classify_auth" ]; then
          timeout 5s env DISPLAY="$classify_display" XAUTHORITY="$classify_auth" glxinfo -B >/dev/null 2>&1 || classify_render_ok=0
        else
          timeout 5s env -u XAUTHORITY DISPLAY="$classify_display" glxinfo -B >/dev/null 2>&1 || classify_render_ok=0
        fi
      fi
      if [ "$classify_render_ok" -eq 1 ]; then
        export DISPLAY="$classify_display"
        if [ -n "$classify_auth" ]; then export XAUTHORITY="$classify_auth"; else unset XAUTHORITY; fi
        printf '[classify env] Display from your desktop session: %s\n' "$DISPLAY" >&2
        break
      fi
    done < <(PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
import os
from pathlib import Path
seen = set()
for proc in sorted((p for p in Path('/proc').iterdir() if p.name.isdigit()), key=lambda p: int(p.name)):
    try:
        if proc.stat().st_uid != os.getuid():
            continue
        argv = (proc / 'cmdline').read_bytes().split(b'\0')
        if Path(argv[0].decode()).name not in {'gnome-shell', 'gnome-session-binary', 'Xwayland'}:
            continue
        env = dict(item.split(b'=', 1) for item in (proc / 'environ').read_bytes().split(b'\0') if b'=' in item)
        display, auth = env.get(b'DISPLAY', b''), env.get(b'XAUTHORITY', b'')
        if not display.startswith(b':') or (display, auth) in seen:
            continue
        seen.add((display, auth))
        os.write(1, display + b'\0' + auth + b'\0')
    except (OSError, ValueError, UnicodeError):
        continue
PY
)
  fi
  return 0
}
classify_runtime_env
unset -f classify_runtime_env
