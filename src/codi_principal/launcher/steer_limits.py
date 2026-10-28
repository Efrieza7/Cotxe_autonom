#!/usr/bin/env python3
"""Gira les rodes al màxim cap a un costat o l'altre i fa avançar el cotxe.

Fa servir la calibració de `full_car.launch.py` (SERVO_*, braços, pins i
inversions dels motors), la mateixa conversió que `steering`, i mostra el gir
a la matriu de LEDs de la Sense HAT (fletxa cap al costat on giren les rodes;
vermella mentre els motors van) i al terminal (pols, angle de les rodes, radi
de gir i PWM dels motors). Els motors van amb PWM fix, sense PID:

    ros2 run my_pakage steer_limits     # amb el workspace carregat

Tecles:
    a       rodes al màxim a l'ESQUERRA
    d       rodes al màxim a la DRETA
    s       rodes rectes (centre)
    w       motors endavant amb el PWM actual
    espai   atura els motors
    + / -   puja / baixa el PWM un 5 %
    x       surt (atura els motors i deixa les rodes rectes)

El servei status-led també dibuixa a la matriu: atura'l abans
(sudo systemctl stop status-led) o la fletxa parpellejarà.
"""
import argparse
import ast
import glob
import math
import os
import struct
import subprocess
import sys
import termios
import time
import tty

try:
    from nodes.control.servo_pwm import ServoPwm
except ImportError:
    # Executat directament (python3 steer_limits.py), sense el workspace.
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from nodes.control.servo_pwm import ServoPwm


def default_launch_file() -> str:
    """full_car.launch.py del codi font o, amb ros2 run, l'instal·lat."""
    source = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch', 'full_car.launch.py')
    if os.path.exists(source):
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('my_pakage'), 'launch', 'full_car.launch.py')
    except Exception:
        return source


CONSTANTS = (
    'WHEELBASE', 'SERVO_PIN', 'SERVO_MIN_PULSE_US', 'SERVO_CENTER_PULSE_US',
    'SERVO_MAX_PULSE_US', 'SERVO_MAX_ANGLE', 'SERVO_INVERT', 'SERVO_ARM_MM', 'KNUCKLE_ARM_MM',
    'MOTOR_AIN1_PIN', 'MOTOR_AIN2_PIN', 'MOTOR_BIN1_PIN', 'MOTOR_BIN2_PIN',
    'INVERT_MOTOR_A', 'INVERT_MOTOR_B',
)
# PWM dels motors (%): inicial i màxim que es deixa posar amb '+'.
START_DUTY = 30.0
MAX_DUTY = 60.0
DUTY_STEP = 5.0

OFF = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 120, 255)
ORANGE = (255, 120, 0)
RED = (255, 0, 0)
# Fletxa cap a l'esquerra (8x8, 1 = encès); la de la dreta és el mirall.
ARROW_LEFT = [
    '00010000',
    '00110000',
    '01111111',
    '11111111',
    '11111111',
    '01111111',
    '00110000',
    '00010000',
]
STRAIGHT = [
    '00011000',
    '00111100',
    '01111110',
    '00011000',
    '00011000',
    '00011000',
    '00011000',
    '00011000',
]


def load_calibration(path: str) -> dict:
    """Llegeix les constants del servo de full_car.launch.py sense executar-lo."""
    with open(path) as f:
        tree = ast.parse(f.read(), path)
    values = {}
    for node in tree.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name) and node.targets[0].id in CONSTANTS):
            values[node.targets[0].id] = eval(compile(ast.Expression(node.value), path, 'eval'), {'math': math})
    missing = [name for name in CONSTANTS if name not in values]
    if missing:
        raise RuntimeError(f'Falten constants a {path}: {", ".join(missing)}')
    return values


class Steering:
    """Mateixa conversió angle de rodes -> pols que el node `steering`."""

    def __init__(self, cal: dict):
        self.cal = cal
        self.arm_ratio = cal['SERVO_ARM_MM'] / cal['KNUCKLE_ARM_MM']
        self.max_angle = math.asin(min(1.0, self.arm_ratio * math.sin(min(cal['SERVO_MAX_ANGLE'], math.pi / 2))))

    def pulse_us(self, wheel_angle: float) -> float:
        servo = math.asin(max(-1.0, min(1.0, math.sin(wheel_angle) / self.arm_ratio)))
        t = max(-1.0, min(1.0, servo / self.cal['SERVO_MAX_ANGLE']))
        if self.cal['SERVO_INVERT']:
            t = -t
        center = self.cal['SERVO_CENTER_PULSE_US']
        if t >= 0.0:
            return center + t * (self.cal['SERVO_MAX_PULSE_US'] - center)
        return center + t * (center - self.cal['SERVO_MIN_PULSE_US'])


class SenseHatMatrix:
    """Matriu 8x8 de la Sense HAT, escrivint directament al framebuffer."""

    def __init__(self):
        self.device = None
        for name_file in glob.glob('/sys/class/graphics/fb*/name'):
            with open(name_file) as f:
                if f.read().strip() == 'RPi-Sense FB':
                    self.device = '/dev/' + os.path.basename(os.path.dirname(name_file))
        if self.device is None:
            raise RuntimeError("no es troba el framebuffer de la Sense HAT ('RPi-Sense FB')")
        self.fb = open(self.device, 'wb')

    def draw(self, rows, color):
        r, g, b = color
        on = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)  # RGB565
        pixels = [on if c == '1' else 0 for row in rows for c in row]
        self.fb.seek(0)
        self.fb.write(struct.pack('<64H', *pixels))
        self.fb.flush()

    def close(self):
        self.draw(['0' * 8] * 8, OFF)
        self.fb.close()


def read_key() -> str:
    """Llegeix una tecla sense esperar l'Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def status_led_active() -> bool:
    try:
        return subprocess.run(['systemctl', 'is-active', '--quiet', 'status-led'], check=False).returncode == 0
    except FileNotFoundError:
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--launch', default=default_launch_file(), help='fitxer d\'on es llegeix la calibració')
    args, _ = parser.parse_known_args()  # ignora els arguments que afegeix ros2 run

    if not sys.stdin.isatty():
        sys.exit('Cal executar-ho des d\'un terminal (llegeix el teclat).')

    cal = load_calibration(args.launch)
    steering = Steering(cal)
    max_deg = math.degrees(steering.max_angle)
    min_radius = cal['WHEELBASE'] / math.tan(steering.max_angle)
    print(f'Calibració de {os.path.abspath(args.launch)}')
    print(f'Angle màxim de les rodes: ±{max_deg:.1f} deg -> radi de gir mínim (eix posterior) '
          f'{min_radius:.2f} m')

    servo = None
    try:
        servo = ServoPwm(int(cal['SERVO_PIN']), steering.pulse_us(0.0))
        print(f'Servo al GPIO {cal["SERVO_PIN"]} amb PWM de {servo.kind}.')
    except Exception as e:
        print(f'No es pot generar el senyal del servo ({e}). No es mourà cap servo.')

    motors = None
    try:
        from nodes.control.speed_control import GPIO, DRV8833Motor
        if GPIO is None:
            raise RuntimeError('RPi.GPIO no disponible')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        motors = [
            DRV8833Motor(int(cal['MOTOR_AIN1_PIN']), int(cal['MOTOR_AIN2_PIN']), invert=bool(cal['INVERT_MOTOR_A'])),
            DRV8833Motor(int(cal['MOTOR_BIN1_PIN']), int(cal['MOTOR_BIN2_PIN']), invert=bool(cal['INVERT_MOTOR_B'])),
        ]
    except Exception as e:
        print(f'No es poden moure els motors ({e}). Carrega el workspace (source install/setup.bash).')

    matrix = None
    try:
        matrix = SenseHatMatrix()
        if status_led_active():
            print('Avís: el servei status-led està actiu i també dibuixa a la matriu '
                  '(sudo systemctl stop status-led).')
    except Exception as e:
        print(f'Sense matriu de LEDs ({e}): només es mostra al terminal.')

    positions = {
        'a': ('ESQUERRA', steering.max_angle, ARROW_LEFT, BLUE),
        'd': ('DRETA', -steering.max_angle, [row[::-1] for row in ARROW_LEFT], ORANGE),
        's': ('RECTE', 0.0, STRAIGHT, GREEN),
    }
    print(__doc__.split('Tecles:')[1].split('El servei')[0].rstrip())

    state = {'steer': 's', 'driving': False, 'duty': START_DUTY}

    def set_motors(duty):
        if motors is not None:
            for motor in motors:
                motor.set(duty)

    def show():
        name, angle, rows, color = positions[state['steer']]
        pulse = steering.pulse_us(angle)
        if servo is not None:
            servo.set_pulse(pulse)
        set_motors(state['duty'] if state['driving'] else 0.0)
        if matrix is not None:
            matrix.draw(rows, RED if state['driving'] else color)
        radius = f'{cal["WHEELBASE"] / math.tan(abs(angle)):.2f} m' if angle else 'recte'
        drive = f'ENDAVANT {state["duty"]:3.0f} %' if state['driving'] else f'aturat (PWM {state["duty"]:.0f} %)'
        print(f'\r{name:8s} rodes {math.degrees(angle):+5.1f} deg | pols {pulse:6.0f} us | radi {radius:8s}'
              f'| motors {drive:20s}', end='', flush=True)

    try:
        show()
        while True:
            key = read_key().lower()
            if key in ('x', '\x03'):  # x o Ctrl+C
                break
            if key in positions:
                state['steer'] = key
            elif key == 'w':
                state['driving'] = True
            elif key == ' ':
                state['driving'] = False
            elif key in ('+', '='):
                state['duty'] = min(MAX_DUTY, state['duty'] + DUTY_STEP)
            elif key == '-':
                state['duty'] = max(0.0, state['duty'] - DUTY_STEP)
            else:
                continue
            show()
    finally:
        if motors is not None:
            for motor in motors:
                motor.stop()
        if servo is not None:
            servo.set_pulse(steering.pulse_us(0.0))
            time.sleep(0.5)  # temps perquè el servo arribi al centre
            servo.stop()
        if matrix is not None:
            matrix.close()
        print('\nMotors aturats i rodes rectes.')


if __name__ == '__main__':
    main()
