#!/usr/bin/env python3
"""Espera que es premi el botó del joystick de la Sense HAT i engega el cotxe.

En prémer el botó del mig del joystick s'executa
`ros2 launch my_pakage full_car.launch.py` (amb l'entorn de `install/setup.bash`).

Mentre espera el botó, els LEDs de la Sense HAT es posen blaus: crea el fitxer
WAITING_FLAG (amb el seu PID) i `status_led` el mira.

Opcions:
  --no-wait  no espera el botó, engega directament
  --loop     (el fa servir el servei `raspberry/car-launcher.service`)
             torna a esperar el botó quan s'acaba el launch, i si es prem el
             botó amb el cotxe engegat s'atura el launch

S'engega sol en arrencar la Raspberry amb el servei
`raspberry/car-launcher.service` (sudo ./raspberry/install_car_launcher.sh).
"""
import glob
import os
import select
import signal
import struct
import subprocess
import sys
import time

# status_led posa els LEDs blaus mentre existeix aquest fitxer i el procés que
# hi ha escrit segueix viu.
WAITING_FLAG = '/tmp/cotxe_esperant_boto'

JOYSTICK_NAME = 'Raspberry Pi Sense HAT Joystick'
# struct input_event de Linux (64 bits): timeval (2 x long), type, code, value.
EVENT_FORMAT = 'llHHi'
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
EV_KEY = 1
KEY_ENTER = 28  # botó del mig del joystick
KEY_PRESSED = 1

# Després d'engegar, un temps sense fer cas del botó perquè un doble clic no
# aturi el cotxe just en engegar-lo.
IGNORE_AFTER_START_SEC = 2.0
# Temps que es deixa al launch per aturar-se amb SIGINT (motors a 0).
STOP_TIMEOUT_SEC = 15.0


def find_workspace_root(start_path: str = None) -> str | None:
    path = os.path.abspath(start_path or os.path.dirname(__file__))
    while True:
        if os.path.isdir(os.path.join(path, 'install')):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            return None
        path = parent


def open_joystick():
    """Obre el /dev/input/event* del joystick de la Sense HAT (o None)."""
    for name_file in glob.glob('/sys/class/input/event*/device/name'):
        with open(name_file) as f:
            if f.read().strip() == JOYSTICK_NAME:
                event = name_file.split('/')[4]
                return os.open('/dev/input/' + event, os.O_RDONLY | os.O_NONBLOCK)
    return None


def button_pressed(fd, timeout):
    """Espera fins a `timeout` s. True si s'ha premut el botó del mig."""
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return False
    pressed = False
    while True:
        try:
            data = os.read(fd, EVENT_SIZE * 16)
        except BlockingIOError:
            break
        if not data:
            break
        for i in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
            _, _, ev_type, code, value = struct.unpack_from(EVENT_FORMAT, data, i)
            if ev_type == EV_KEY and code == KEY_ENTER and value == KEY_PRESSED:
                pressed = True
    return pressed


def discard_events(fd):
    """Llença els esdeveniments pendents (p. ex. clics mentre el cotxe anava)."""
    while button_pressed(fd, 0):
        pass


def set_waiting(waiting):
    if waiting:
        with open(WAITING_FLAG, 'w') as f:
            f.write(str(os.getpid()))
    else:
        try:
            os.remove(WAITING_FLAG)
        except FileNotFoundError:
            pass


def wait_for_joystick_click(fd):
    """Bloqueja fins que es prem el botó (o Enter si no hi ha Sense HAT)."""
    set_waiting(True)
    try:
        if fd is None:
            print('No es troba el joystick de la Sense HAT: prem Enter per engegar...')
            input()
            return
        print('Esperant el botó del joystick per engegar el cotxe...', flush=True)
        discard_events(fd)
        while not button_pressed(fd, 1.0):
            pass
    finally:
        set_waiting(False)


def stop_launch(proc):
    """Atura el launch com amb Ctrl+C (els nodes posen els motors a 0)."""
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(STOP_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def run_launch(cmd, fd, stop_with_button):
    """Executa el launch. Si stop_with_button, el botó l'atura."""
    print('Engegant: ' + cmd, flush=True)
    # Sessió pròpia: així es poden matar tots els nodes junts si cal.
    proc = subprocess.Popen(['bash', '-c', cmd], start_new_session=True)
    started = time.monotonic()
    try:
        while proc.poll() is None:
            if not stop_with_button or fd is None:
                time.sleep(0.5)
                continue
            if button_pressed(fd, 0.5) and time.monotonic() - started > IGNORE_AFTER_START_SEC:
                print('Botó premut: aturant el cotxe...', flush=True)
                stop_launch(proc)
    except KeyboardInterrupt:
        stop_launch(proc)
        raise
    return proc.returncode


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    # --no-wait: no espera el joystick; --wait: l'espera (per defecte)
    wait_for_button = True
    if '--no-wait' in argv:
        wait_for_button = False
        argv.remove('--no-wait')
    elif '--wait' in argv:
        argv.remove('--wait')
    loop = '--loop' in argv
    if loop:
        argv.remove('--loop')

    ws = find_workspace_root()
    if not ws:
        print('Workspace root (containing `install/`) not found. Run from workspace root or install first.', file=sys.stderr)
        return 2

    setup_bash = os.path.join(ws, 'install', 'setup.bash')
    if not os.path.isfile(setup_bash):
        print(f'File not found: {setup_bash}. Build and install the workspace first.', file=sys.stderr)
        return 3

    cmd = f"source '{setup_bash}' && exec ros2 launch my_pakage full_car.launch.py {' '.join(argv)}"
    fd = open_joystick() if wait_for_button or loop else None
    try:
        while True:
            if wait_for_button:
                wait_for_joystick_click(fd)
            code = run_launch(cmd, fd, stop_with_button=loop)
            if not loop:
                return code
            print(f'El launch ha acabat (codi {code}).', flush=True)
            wait_for_button = True
    except KeyboardInterrupt:
        return 0
    finally:
        set_waiting(False)
        if fd is not None:
            os.close(fd)


if __name__ == '__main__':
    raise SystemExit(main())
