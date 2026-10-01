#!/usr/bin/env python3
"""Own complete-launch process groups even after ros2 launch has exited."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / 'log/classify_runtime'
RECORD = RUNTIME / 'launch.pid'


def processes():
    rows = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            if path.stat().st_uid != os.getuid():
                continue
            stat = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            if stat[0] == 'Z':
                continue
            argv = (path / 'cmdline').read_bytes().split(b'\0')
            env = dict(item.split(b'=', 1) for item in (path / 'environ').read_bytes().split(b'\0') if b'=' in item)
            rows[int(path.name)] = {
                'pgid': int(stat[2]), 'sid': int(stat[3]), 'start': int(stat[19]),
                'argv': [s.decode(errors='replace') for s in argv if s], 'env': env,
            }
        except (OSError, ValueError, IndexError):
            continue
    return rows


def owned(row):
    env = row['env']
    return (env.get(b'CLASSIFY_WS_ROOT') == os.fsencode(ROOT) or
            env.get(b'ROS_LOG_DIR') == os.fsencode(RUNTIME))


def groups(rows):
    result = set()
    for row in rows.values():
        # setsid makes this an isolated launch session, never the user's shell.
        if owned(row) and row['pgid'] == row['sid'] and row['pgid'] != os.getpgrp():
            result.add(row['pgid'])
    return result


def stop(expected_pid, quiet):
    initial = RECORD.read_text().strip() if RECORD.exists() else None
    if expected_pid is not None and (not initial or initial.split()[0] != str(expected_pid)):
        # A previous wrapper's EXIT trap must never stop a newer launch.
        return 0
    rows = processes()
    targets = groups(rows)
    if initial:
        try:
            pid, start = map(int, initial.split())
        except ValueError:
            print('Invalid launch PID record; no process was signalled', file=sys.stderr)
            return 1
        leader = rows.get(pid)
        if leader and (leader['start'] != start or not owned(leader) or leader['pgid'] != pid):
            print('PID record belongs to another process; no process was signalled', file=sys.stderr)
            return 1
        if leader:
            targets.add(pid)
    # Verify every live group member before signalling an entire group.
    for group in targets:
        if any(not owned(row) for row in rows.values() if row['pgid'] == group):
            print(f'Process group {group} includes an unowned process; not stopped', file=sys.stderr)
            return 1
    if not quiet:
        print('Stopping workspace launch groups: ' + ', '.join(map(str, sorted(targets))) if targets
              else 'No complete simulation remains for this workspace.', flush=True)
    for group in targets:
        try:
            os.killpg(group, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 10
    live = targets
    while live and time.monotonic() < deadline:
        live = {row['pgid'] for row in processes().values()} & targets
        if live:
            time.sleep(.2)
    # Membership was verified; recheck ownership before forced cleanup.
    current = processes()
    for group in live:
        if all(owned(row) for row in current.values() if row['pgid'] == group):
            try:
                os.killpg(group, signal.SIGKILL)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + 3
    while live and time.monotonic() < deadline:
        live = {row['pgid'] for row in processes().values()} & targets
        if live:
            time.sleep(.1)
    if live:
        print(f'Cleanup incomplete for groups {live}; PID record retained', file=sys.stderr)
        return 1
    if initial and RECORD.exists() and RECORD.read_text().strip() == initial:
        RECORD.unlink()
    if not quiet:
        print('Classification simulation stopped; next launch loads the initial world.', flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--quiet', action='store_true')
    parser.add_argument('--expected-pid', type=int)
    args = parser.parse_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    if args.check:
        return 1 if groups(processes()) else 0
    # Concurrent stop calls and old EXIT traps serialize with launch startup.
    with (RUNTIME / 'lifecycle.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return stop(args.expected_pid, args.quiet)


if __name__ == '__main__':
    raise SystemExit(main())
