#!/usr/bin/env python3
"""Posa tots els GPIO de la Raspberry a 0 (estat segur).

No depèn de ROS, així es pot executar en arrencar/apagar la Raspberry amb el
servei `raspberry/gpio-reset.service`, o a mà si un motor s'ha quedat engegat:

    sudo python3 gpio_reset.py        # sense ROS
    ros2 run my_pakage gpio_reset     # amb el workspace carregat

- Pins que són SORTIDES del cotxe (motors): sortida a 0.
- Servo (GPIO 18): fa servir el PWM de maquinari, que no és un GPIO normal.
  No es toca el pin (si es fes sortida perdria la funció de PWM fins a
  reiniciar); s'atura el senyal desactivant els canals de /sys/class/pwm.
- La resta de pins lliures: entrada amb pull-down (es llegeixen com a 0).
  No es posen com a sortida perquè si hi ha connectat un sensor que treu
  tensió (p. ex. l'encoder), una sortida a 0 el curtcircuitaria.
- Pins reservats: no es toquen (EEPROM dels HAT, I2C de la IMU/Sense HAT i
  UART).
"""
import argparse
import glob
import sys

# Pins BCM. Han de coincidir amb full_car.launch.py.
OUTPUT_PINS = [
    5, 6, 26, 16,     # DRV8833: AIN1, AIN2, BIN1, BIN2
]
RESERVED_PINS = [
    18,               # servo de direcció: PWM de maquinari (dtoverlay=pwm)
    0, 1,             # ID EEPROM dels HAT
    2, 3,             # I2C (IMU / Sense HAT)
    7, 8, 9, 10, 11,  # SPI0 (actiu a Ubuntu: el kernel té ocupat el GPIO 7). La
                      # Sense HAT el fa servir (amb el 25) per programar els LEDs
    25,               # Sense HAT: reset del microcontrolador dels LEDs
    23, 24,           # Sense HAT: interrupcions de la IMU (són sortides de la IMU)
    14, 15,           # UART
]
ALL_PINS = range(0, 28)  # GPIO 0..27 del connector de 40 pins


def parse_pins(text):
    return [int(p) for p in text.split(',') if p.strip()]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--outputs', type=parse_pins, default=OUTPUT_PINS,
                        help='pins (BCM) a posar com a sortida a 0, separats per comes')
    parser.add_argument('--reserved', type=parse_pins, default=RESERVED_PINS,
                        help='pins (BCM) que no es toquen, separats per comes')
    args = parser.parse_args(argv)

    # Servo: atura el PWM de maquinari (segueix generant polsos encara que
    # el programa que l'ha engegat s'hagi acabat).
    for enable in glob.glob('/sys/class/pwm/pwmchip*/pwm*/enable'):
        try:
            with open(enable, 'w') as f:
                f.write('0')
        except OSError as e:
            print(f'gpio_reset: no es pot aturar {enable}: {e}', file=sys.stderr)

    try:
        import RPi.GPIO as GPIO
    except (ImportError, RuntimeError) as e:
        print(f'gpio_reset: no es pot carregar RPi.GPIO ({e})', file=sys.stderr)
        return 1

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    errors = 0
    outputs = [p for p in args.outputs if p not in args.reserved]
    inputs = [p for p in ALL_PINS if p not in outputs and p not in args.reserved]
    for pin in outputs:
        try:
            # Primer el pull-down: quan aquest programa s'acaba, el kernel torna
            # el pin a entrada, i sense pull-down es quedaria amb el pull-up per
            # defecte dels GPIO 0-8 (motor engegat).
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(pin, GPIO.LOW)
        except Exception as e:  # p. ex. pin ocupat per un altre procés
            errors += 1
            print(f'gpio_reset: GPIO {pin} (sortida): {e}', file=sys.stderr)
    for pin in inputs:
        try:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        except Exception as e:
            errors += 1
            print(f'gpio_reset: GPIO {pin} (entrada): {e}', file=sys.stderr)

    # No es fa GPIO.cleanup(): tornaria les sortides a entrades. Els pins es
    # queden a 0 quan s'acaba el programa.
    print(f'gpio_reset: sortides a 0 {outputs}; entrades amb pull-down {inputs}; '
          f'sense tocar {sorted(args.reserved)}')
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
