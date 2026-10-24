"""Senyal del servo de la direcció (50 Hz, amplada de pols en microsegons).

Als pins amb PWM de maquinari de la Raspberry Pi 4 (GPIO 12/18 = canal 0,
13/19 = canal 1) el senyal el genera el maquinari, a través de
/sys/class/pwm: els polsos són exactes encara que la CPU vagi al 100 %.
Cal activar-lo al config.txt (ho fa `raspberry/install_gpio_reset.sh`):

    dtoverlay=pwm,pin=18,func=2

A qualsevol altre pin, o si el PWM de maquinari no està activat, es fa
servir el PWM per software de RPi.GPIO (rpi-lgpio), que té jitter quan la
CPU està carregada i fa que el servo tingui espasmes.

Fan servir aquesta classe `steering`, `test_servo`, `test_motors` i
`calibrate_servo`.
"""
import glob
import os
import time

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    GPIO = None

PWM_FREQUENCY_HZ = 50.0
PERIOD_NS = int(1e9 / PWM_FREQUENCY_HZ)

# Pin BCM -> canal del PWM0 de la Raspberry Pi 4 (bloc fe20c000.pwm).
HW_PWM_CHANNEL = {12: 0, 18: 0, 13: 1, 19: 1}


def _find_pwm_chip():
    """Directori /sys/class/pwm/pwmchipN del PWM0 (o None si no està activat)."""
    chips = sorted(glob.glob('/sys/class/pwm/pwmchip*'))
    for chip in chips:
        device = os.path.realpath(os.path.join(chip, 'device'))
        if 'fe20c000.pwm' in device or '7e20c000.pwm' in device:
            return chip
    # Si només n'hi ha un (overlay pwm i res més), és aquest.
    return chips[0] if len(chips) == 1 else None


class ServoPwm:
    """Servo a `pin` (BCM). `set_pulse(us)` canvia l'amplada del pols."""

    def __init__(self, pin, pulse_us, warn=print):
        self.pin = pin
        self.kind = None
        self.last_pulse = None
        self._pwm_dir = None
        self._sw_pwm = None

        channel = HW_PWM_CHANNEL.get(pin)
        chip = _find_pwm_chip() if channel is not None else None
        if chip is not None:
            self._start_hardware(chip, channel, pulse_us)
            self.kind = 'maquinari'
            return

        if channel is not None:
            warn(f'GPIO {pin} té PWM de maquinari però no està activat (falta '
                 f'"dtoverlay=pwm,pin={pin},func=2" al config.txt): es fa servir el PWM '
                 'per software, i el servo pot tenir espasmes amb la CPU carregada.')
        if GPIO is None:
            raise RuntimeError('ni PWM de maquinari ni RPi.GPIO disponibles')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self._sw_pwm = GPIO.PWM(pin, PWM_FREQUENCY_HZ)
        self._sw_pwm.start(self._duty(pulse_us))
        self.last_pulse = round(pulse_us)
        self.kind = 'software'

    @staticmethod
    def _duty(pulse_us):
        return 100.0 * pulse_us * PWM_FREQUENCY_HZ / 1e6

    # ---- PWM de maquinari (sysfs) ----
    def _write(self, name, value):
        # Just després d'exportar el canal, udev triga un moment a donar
        # permisos al grup gpio (regla 99-pwm.rules).
        path = os.path.join(self._pwm_dir, name)
        for _ in range(50):
            try:
                with open(path, 'w') as f:
                    f.write(str(value))
                return
            except (PermissionError, FileNotFoundError):
                time.sleep(0.02)
        with open(path, 'w') as f:
            f.write(str(value))

    def _start_hardware(self, chip, channel, pulse_us):
        self._pwm_dir = os.path.join(chip, f'pwm{channel}')
        if not os.path.isdir(self._pwm_dir):
            with open(os.path.join(chip, 'export'), 'w') as f:
                f.write(str(channel))
        # Primer el període: amb el canal acabat d'exportar és 0, i el driver
        # (pwm-bcm2835) rebutja qualsevol altra escriptura (EINVAL) fins que
        # n'hi ha un de vàlid.
        self._write('period', PERIOD_NS)
        self._write('duty_cycle', int(round(pulse_us)) * 1000)
        self._write('enable', 1)
        self.last_pulse = round(pulse_us)

    # ---- comú ----
    def set_pulse(self, pulse_us):
        """Canvia el pols (us). Només s'envia si canvia (resolució de 1 us)."""
        pulse = round(pulse_us)
        if pulse == self.last_pulse:
            return
        if self._pwm_dir is not None:
            self._write('duty_cycle', pulse * 1000)
        else:
            self._sw_pwm.ChangeDutyCycle(self._duty(pulse))
        self.last_pulse = pulse

    def stop(self):
        """Atura el senyal (línia a 0: el servo queda sense força)."""
        if self._pwm_dir is not None:
            try:
                self._write('enable', 0)
            except OSError:
                pass
            self._pwm_dir = None
        elif self._sw_pwm is not None:
            try:
                self._sw_pwm.stop()
                GPIO.output(self.pin, GPIO.LOW)
            except Exception:
                pass
            self._sw_pwm = None
