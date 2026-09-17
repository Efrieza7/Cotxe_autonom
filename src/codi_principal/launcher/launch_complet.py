#!/usr/bin/env python3
"""Simple launcher helper: sources workspace and runs the combined launch.

This script finds the workspace root (a parent directory containing an `install`
folder), sources `install/setup.bash` and then runs `ros2 launch my_pakage
complet_code.launch.py` in a single bash invocation so the sourced environment
applies to the `ros2` command.
"""
import os
import subprocess
import sys
import time


def find_workspace_root(start_path: str = None) -> str | None:
    path = os.path.abspath(start_path or os.path.dirname(__file__))
    while True:
        if os.path.isdir(os.path.join(path, 'install')):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            return None
        path = parent


def main(argv=None):
    argv = argv or sys.argv[1:]
    # CLI flags: --no-wait to skip waiting for joystick, --wait to force waiting
    wait_for_button = True
    if '--no-wait' in argv:
        wait_for_button = False
        argv.remove('--no-wait')
    elif '--wait' in argv:
        wait_for_button = True
        argv.remove('--wait')
    ws = find_workspace_root()
    if not ws:
        print('Workspace root (containing `install/`) not found. Run from workspace root or install first.', file=sys.stderr)
        return 2

    setup_bash = os.path.join(ws, 'install', 'setup.bash')
    if not os.path.isfile(setup_bash):
        print(f'File not found: {setup_bash}. Build and install the workspace first.', file=sys.stderr)
        return 3

    cmd = f"source '{setup_bash}' && ros2 launch my_pakage complet_code.launch.py {' '.join(argv)}"
    if wait_for_button:
        wait_for_joystick_click()
    try:
        # Use bash -c so `source` affects the same shell that runs ros2
        return subprocess.call(['bash', '-c', cmd])
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    raise SystemExit(main())


def wait_for_joystick_click():
    """Block until the Sense HAT joystick is clicked.

    If the `sense_hat` library is not available, fall back to waiting for
    the user to press Enter on the keyboard.
    """
    try:
        from sense_hat import SenseHat
    except Exception:
        try:
            print('Sense HAT library not available: press Enter to continue...')
            input()
        except KeyboardInterrupt:
            pass
        return

    sh = SenseHat()
    print('Waiting for Sense HAT joystick press...')
    sh.clear()
    try:
        while True:
            events = sh.stick.get_events()
            for e in events:
                if getattr(e, 'action', None) == 'pressed':
                    sh.clear()
                    return
            time.sleep(0.05)
    except KeyboardInterrupt:
        sh.clear()
        return
