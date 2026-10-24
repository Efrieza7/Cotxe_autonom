#!/usr/bin/env python3
"""Calibratge del servo de la direcció: troba el zero i el rang de moviment.

Mou el servo amb el teclat i guarda tres punts: el tope esquerre, el centre
(rodes rectes) i el tope dret. En sortir mostra els valors per copiar a
`full_car.launch.py`. No depèn de ROS:

    sudo python3 calibrate_servo.py      # sense ROS
    ros2 run my_pakage calibrate_servo   # amb el workspace carregat

Tecles:
    a / d   -10 us / +10 us
    q / e    -1 us /  +1 us
    1       guarda la posició actual com a TOPE ESQUERRE (rodes a l'esquerra)
    2       guarda la posició actual com a CENTRE (rodes rectes)
    3       guarda la posició actual com a TOPE DRET (rodes a la dreta)
    x       surt (deixa el servo al centre i mostra el resultat)

Els topes s'han de buscar fins on les rodes deixen de girar o el mecanisme
fa força (el servo brunzeix). Allà no s'hi ha de deixar el servo: cal tornar
enrere uns 20-30 us abans de guardar, o es pot cremar.
"""
import argparse
import os
import sys
import termios
import time
import tty

try:
    from nodes.control.servo_pwm import ServoPwm
except ImportError:
    # Executat directament (python3 calibrate_servo.py), sense el workspace.
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from nodes.control.servo_pwm import ServoPwm

# Límits absoluts per no forçar el servo fora del que accepta un MG90S.
ABS_MIN_PULSE_US = 500.0
ABS_MAX_PULSE_US = 2500.0
# Supòsit actual del codi: 500 us de pols = 45 deg de servo.
US_PER_DEG = 500.0 / 45.0


def read_key() -> str:
    """Llegeix una tecla sense esperar l'Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_result(points: dict) -> None:
    print('\n--- Resultat ---')
    for name in ('esquerra', 'centre', 'dreta'):
        value = points.get(name)
        print(f'  {name:9s}: {value:.0f} us' if value is not None else f'  {name:9s}: (no guardat)')

    if any(points.get(name) is None for name in ('esquerra', 'centre', 'dreta')):
        print('\nFalten punts: no es pot calcular la configuració.')
        return

    left, center, right = points['esquerra'], points['centre'], points['dreta']
    if not (min(left, right) < center < max(left, right)):
        print('\nEl centre no queda entre els dos topes: torna-ho a provar.')
        return

    # steering suposa el mateix angle de servo als dos costats, així que es fa
    # servir el recorregut del costat més curt per als dos.
    half = min(abs(left - center), abs(right - center))
    invert = left < right  # steering: pols més llarg = esquerra
    servo_deg = half / US_PER_DEG

    print(f'\nRecorregut útil: ±{half:.0f} us respecte del centre '
          f'(esquerra {abs(left - center):.0f} us, dreta {abs(right - center):.0f} us)')
    print('\nCopia això a full_car.launch.py:\n')
    print(f'SERVO_MIN_PULSE_US = {center - half:.1f}')
    print(f'SERVO_CENTER_PULSE_US = {center:.1f}')
    print(f'SERVO_MAX_PULSE_US = {center + half:.1f}')
    print(f'SERVO_MAX_ANGLE = math.radians({servo_deg:.1f})  # estimat; millor mesurar-lo')
    print(f'SERVO_INVERT = {invert}')
    print('\nSERVO_MAX_ANGLE suposa 500 us = 45 deg. Si pots, mesura amb un transportador '
          "l'angle del braç del servo entre el centre i un tope i posa aquest valor.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--pin', type=int, default=18, help='pin BCM del servo (per defecte 18)')
    parser.add_argument('--start', type=float, default=1500.0, help='pols inicial en us (per defecte 1500)')
    args, _ = parser.parse_known_args()  # ignora els arguments que afegeix ros2 run

    if not sys.stdin.isatty():
        sys.exit('Cal executar-ho des d\'un terminal (llegeix el teclat).')

    pulse = max(ABS_MIN_PULSE_US, min(ABS_MAX_PULSE_US, args.start))
    servo = None
    try:
        servo = ServoPwm(args.pin, pulse)
        print(f'Servo al GPIO {args.pin} amb PWM de {servo.kind}.')
    except Exception as e:
        print(f'No es pot generar el senyal del servo ({e}). '
              'No es mourà cap servo, només es mostren els polsos.')

    points = {}
    steps = {'a': -10.0, 'd': 10.0, 'q': -1.0, 'e': 1.0}
    names = {'1': 'esquerra', '2': 'centre', '3': 'dreta'}

    print(__doc__.split('Tecles:')[1].split('Els topes')[0].rstrip())

    try:
        while True:
            print(f'\rpols = {pulse:6.0f} us   ', end='', flush=True)
            key = read_key()
            if key in ('x', '\x03'):  # x o Ctrl+C
                break
            if key in steps:
                pulse = max(ABS_MIN_PULSE_US, min(ABS_MAX_PULSE_US, pulse + steps[key]))
                if servo is not None:
                    servo.set_pulse(pulse)
            elif key in names:
                points[names[key]] = pulse
                print(f'\n{names[key]} = {pulse:.0f} us')
    finally:
        if servo is not None:
            servo.set_pulse(points.get('centre', args.start))
            time.sleep(0.5)  # temps perquè el servo arribi al centre
            servo.stop()
        print_result(points)


if __name__ == '__main__':
    main()
