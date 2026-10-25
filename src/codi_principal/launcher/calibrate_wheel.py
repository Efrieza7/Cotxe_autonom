#!/usr/bin/env python3
"""Calibratge de l'encoder: polsos per volta de roda i distància per pols.

Fa girar les rodes a poc a poc i compta els polsos de l'encoder. Tu marques
quan la roda ha fet una volta exacta (amb una marca de cinta a la roda i una
referència fixa al xassís). Com més voltes, més precís. En sortir mostra els
valors per copiar a `full_car.launch.py`. Fa servir les classes del motor i
de l'encoder de `speed_control`, així que cal tenir ROS carregat (no s'ha
d'executar alhora que full_car, fan servir els mateixos pins):

    ros2 run my_pakage calibrate_wheel
    ros2 run my_pakage calibrate_wheel --duty 15   # si gira massa de pressa

Tecles:
    espai   engega / atura el gir continu (a `duty`)
    f / b   empenteta endavant / enrere (ajust fi fins a la marca)
    + / -   puja / baixa el duty del gir continu
    0       posa el comptador a 0 (la marca ha d'estar alineada)
    v       marca una volta completa (amb les rodes aturades a la marca)
    x       surt i mostra el resultat

Procediment: alinea la marca, prem 0, fes girar la roda i atura-la a prop de
la marca; ajusta-la amb f / b i prem v. Repeteix-ho per a 5-10 voltes.
COTXE AIXECAT i les dues rodes lliures: amb el diferencial, si una roda va
més frenada que l'altra, les voltes de cada roda no coincideixen.
"""
import argparse
import math
import os
import sys
import termios
import time
import tty

try:
    from nodes.control.speed_control import GPIO, DRV8833Motor, QuadratureEncoder
except ImportError:
    # Executat directament (python3 calibrate_wheel.py), sense el workspace.
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from nodes.control.speed_control import GPIO, DRV8833Motor, QuadratureEncoder

# Polsos per volta del motor intern i reducció del GA12-N20 (full_car.launch.py).
ENCODER_PULSES_PER_MOTOR_REV = 7.0
GEAR_RATIO = 100.0
# Temps perquè el motor s'aturi del tot abans de llegir els polsos (s).
SETTLE_S = 0.3


def read_key() -> str:
    """Llegeix una tecla sense esperar l'Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_result(turns: int, ticks: int, wheel_diameter: float) -> None:
    print('\n--- Resultat ---')
    if turns == 0 or ticks <= 0:
        print('No s\'ha marcat cap volta: no es pot calcular res.')
        return
    pulses_per_rev = ticks / turns
    wheel_turns_per_motor_turn = ENCODER_PULSES_PER_MOTOR_REV * GEAR_RATIO / pulses_per_rev
    mm_per_pulse = math.pi * wheel_diameter * 1000.0 / pulses_per_rev
    print(f'  {turns} voltes, {ticks} polsos -> {pulses_per_rev:.2f} polsos per volta de roda')
    print(f'  {mm_per_pulse:.4f} mm per pols (roda de {wheel_diameter * 1000.0:.2f} mm)')
    print(f'  {1.0 / (mm_per_pulse / 1000.0):.1f} polsos per metre')
    print('\nCopia això a full_car.launch.py:\n')
    print(f'WHEEL_TURNS_PER_MOTOR_TURN = {wheel_turns_per_motor_turn:.4f}')
    print(f'# PULSES_PER_WHEEL_REV = {pulses_per_rev:.2f}')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duty', type=float, default=20.0, help='PWM del gir continu en %% (per defecte 20)')
    parser.add_argument('--burst-duty', type=float, default=25.0, help='PWM de les empentetes en %% (per defecte 25)')
    parser.add_argument('--burst-s', type=float, default=0.03, help='durada de les empentetes en s (per defecte 0.03)')
    parser.add_argument('--wheel-diameter', type=float, default=0.05436, help='diàmetre de la roda en m')
    # Pins i inversions per defecte: els de full_car.launch.py.
    parser.add_argument('--ain1', type=int, default=5)
    parser.add_argument('--ain2', type=int, default=6)
    parser.add_argument('--bin1', type=int, default=26)
    parser.add_argument('--bin2', type=int, default=16)
    parser.add_argument('--enc-a', type=int, default=27)
    parser.add_argument('--enc-b', type=int, default=17)
    parser.add_argument('--invert-a', type=int, default=1, help='1 = inverteix el motor A (per defecte 1)')
    parser.add_argument('--invert-b', type=int, default=0, help='1 = inverteix el motor B (per defecte 0)')
    parser.add_argument('--encoder-invert', type=int, default=0)
    args, _ = parser.parse_known_args()  # ignora els arguments que afegeix ros2 run

    if not sys.stdin.isatty():
        sys.exit('Cal executar-ho des d\'un terminal (llegeix el teclat).')
    if GPIO is None:
        sys.exit('RPi.GPIO no disponible: aquest calibratge només funciona a la Raspberry.')

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    motors = []
    encoder = None
    try:
        motors = [
            DRV8833Motor(args.ain1, args.ain2, 1000.0, bool(args.invert_a)),
            DRV8833Motor(args.bin1, args.bin2, 1000.0, bool(args.invert_b)),
        ]
        encoder = QuadratureEncoder(args.enc_a, args.enc_b, bool(args.encoder_invert))

        def drive(duty):
            if duty != 0.0:
                encoder.direction = 1 if duty > 0.0 else -1
            for motor in motors:
                motor.set(duty)

        def halt():
            # Fre actiu i esperar que s'aturi: els polsos de l'inèrcia també compten.
            for motor in motors:
                motor.brake()
            time.sleep(SETTLE_S)
            drive(0.0)

        duty = max(0.0, min(100.0, args.duty))
        burst_duty = max(0.0, min(100.0, args.burst_duty))
        running = False
        turns = 0
        marked_ticks = 0

        print(__doc__.split('Tecles:')[1].split('Procediment')[0].rstrip())
        print('\nAlinea la marca i prem 0 per començar.')

        while True:
            ticks = encoder.ticks
            estat = 'GIRANT' if running else 'aturat'
            print(f'\r[{estat}] duty={duty:4.0f}%  polsos={ticks:+7d}  voltes marcades={turns}   ',
                  end='', flush=True)
            key = read_key()
            if key in ('x', '\x03'):  # x o Ctrl+C
                break
            if key == ' ':
                if running:
                    halt()
                else:
                    drive(duty)
                running = not running
            elif key in ('+', '='):
                duty = min(100.0, duty + 2.0)
                if running:
                    drive(duty)
            elif key == '-':
                duty = max(0.0, duty - 2.0)
                if running:
                    drive(duty)
            elif key in ('f', 'b') and not running:
                drive(burst_duty if key == 'f' else -burst_duty)
                time.sleep(args.burst_s)
                halt()
            elif key == '0' and not running:
                encoder.ticks = 0
                turns = 0
                marked_ticks = 0
                print('\nComptador a 0.')
            elif key == 'v' and not running:
                turns += 1
                marked_ticks = encoder.ticks
                print(f'\nVolta {turns}: {marked_ticks} polsos ({marked_ticks / turns:.2f} per volta)')
    finally:
        for motor in motors:
            try:
                motor.stop()
            except Exception:
                pass
        motor = None  # perquè el PWM es destrueixi ara (com a test_motors)
        motors = []
        if encoder is not None:
            try:
                GPIO.cleanup(list(encoder.pins))
            except Exception:
                pass
        print_result(turns, marked_ticks, args.wheel_diameter)


if __name__ == '__main__':
    main()
