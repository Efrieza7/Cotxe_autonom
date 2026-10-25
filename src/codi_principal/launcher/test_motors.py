import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from nodes.control.servo_pwm import PWM_FREQUENCY_HZ, ServoPwm
from nodes.control.speed_control import GPIO, DRV8833Motor, QuadratureEncoder

# Durada màxima de la prova (s). Mai es passa d'aquí, ni canviant paràmetres.
MAX_TEST_S = 10.0


class TestMotors(Node):
    """Prova tipus "missió d'inspecció" de la Formula Student Driverless.

    A la FS, a la inspecció el cotxe està aixecat, les rodes motrius giren i
    la direcció es mou en forma de sinus; després el cotxe frena (EBS) i
    passa a l'estat "finished". Aquí es fa el mateix en menys de 10 s:

    1. preparat  (`ready_s`):  motors aturats, direcció centrada.
    2. inspecció (`drive_s`):  els motors pugen a `duty` en `ramp_s` i la
       direcció fa un sinus (`steer_amplitude`, `steer_period_s`).
    3. frenada   (`brake_s`):  fre actiu del DRV8833 i direcció centrada.
    4. acabat:   tots els pins a 0 i el node s'acaba.

    Cada segon mostra els ticks de l'encoder, que serveixen per comprovar:

    - que els dos motors giren cap endavant (si no, `invert_motor_a` /
      `invert_motor_b`);
    - que els ticks són positius (si no, `encoder_invert`).

    Fa servir els mateixos pins per defecte que `speed_control` i `steering`.
    CAL TENIR EL COTXE AIXECAT: les rodes giren.
    """

    def __init__(self):
        super().__init__('test_motors')

        ain1 = int(self.declare_parameter('ain1_pin', 5).value)
        ain2 = int(self.declare_parameter('ain2_pin', 6).value)
        bin1 = int(self.declare_parameter('bin1_pin', 26).value)
        bin2 = int(self.declare_parameter('bin2_pin', 16).value)
        enc_a = int(self.declare_parameter('enc_a_pin', 27).value)
        enc_b = int(self.declare_parameter('enc_b_pin', 17).value)
        servo_pin = int(self.declare_parameter('servo_pin', 18).value)
        pwm_frequency = float(self.declare_parameter('pwm_frequency_hz', 1000.0).value)
        invert_a = bool(self.declare_parameter('invert_motor_a', False).value)
        invert_b = bool(self.declare_parameter('invert_motor_b', False).value)
        encoder_invert = bool(self.declare_parameter('encoder_invert', False).value)
        self.min_pulse_us = float(self.declare_parameter('min_pulse_us', 1000.0).value)
        self.center_pulse_us = float(self.declare_parameter('center_pulse_us', 1500.0).value)
        self.max_pulse_us = float(self.declare_parameter('max_pulse_us', 2000.0).value)
        self.servo_invert = bool(self.declare_parameter('servo_invert', False).value)
        # Per passar els ticks a m/s (opcional, només informatiu).
        self.wheel_diameter = float(self.declare_parameter('wheel_diameter', 0.05436).value)
        self.pulses_per_rev = float(self.declare_parameter('pulses_per_wheel_rev', 280.0).value)

        # Perfil de la prova.
        self.duty = max(0.0, min(100.0, float(self.declare_parameter('duty', 30.0).value)))
        self.ramp_s = float(self.declare_parameter('ramp_s', 1.5).value)
        self.steer_amplitude = float(self.declare_parameter('steer_amplitude', 0.6).value)
        self.steer_period_s = float(self.declare_parameter('steer_period_s', 2.0).value)
        self.ready_s = float(self.declare_parameter('ready_s', 1.0).value)
        drive_s = float(self.declare_parameter('drive_s', 7.0).value)
        self.brake_s = float(self.declare_parameter('brake_s', 1.0).value)
        self.steer_amplitude = max(0.0, min(1.0, self.steer_amplitude))

        # La prova sencera no pot passar de MAX_TEST_S (deixant 0.5 s de marge).
        self.drive_s = max(0.0, min(drive_s, MAX_TEST_S - 0.5 - self.ready_s - self.brake_s))
        if self.drive_s < drive_s:
            self.get_logger().warn(f'drive_s retallat a {self.drive_s:.1f}s per no passar de {MAX_TEST_S:.0f}s')
        self.end_s = self.ready_s + self.drive_s + self.brake_s

        self.motor_pins = [ain1, ain2, bin1, bin2]
        self.servo_pin = servo_pin
        self.motors = []
        self.encoder = None
        self.servo = None
        self.finished = False
        self.state = None
        # Per als LEDs d'estat (status_led): True mentre giren els motors.
        self.moving_pub = self.create_publisher(Bool, 'car_moving', 10)

        if GPIO is None:
            self.get_logger().error(
                'RPi.GPIO no disponible: aquesta prova només funciona a la Raspberry. '
                'Prova: python3 -c "import RPi.GPIO"'
            )
            self.finished = True
            return

        # Si alguna cosa falla a mig configurar, es deixen els pins a 0
        # abans de sortir (si no, un motor es podria quedar engegat).
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            self.motors = [
                DRV8833Motor(ain1, ain2, pwm_frequency, invert_a),
                DRV8833Motor(bin1, bin2, pwm_frequency, invert_b),
            ]
            self.servo = ServoPwm(servo_pin, self._servo_pulse(0.0), warn=self.get_logger().warn)
        except Exception as e:
            if 'gpiochip' in str(e):
                # lgpio no pot obrir /dev/gpiochip*: falten permisos.
                hint = ('L\'usuari no té permís per obrir /dev/gpiochip*: executa '
                        '`sudo ./raspberry/install_gpio_reset.sh` i torna a iniciar sessió.')
            else:
                hint = 'Comprova que cap altre node (speed_control, steering) els estigui fent servir.'
            self.get_logger().error(
                f'No s\'han pogut configurar els pins ({type(e).__name__}: {e}). {hint}'
            )
            self.release_hardware()
            self.finished = True
            return

        try:
            self.encoder = QuadratureEncoder(enc_a, enc_b, encoder_invert)
        except Exception as e:
            self.get_logger().warn(f'Prova sense encoder: {e}')
            self.encoder = None

        self.get_logger().info(
            f'Inspecció: A=({ain1},{ain2}) B=({bin1},{bin2}) encoder=({enc_a},{enc_b}) '
            f'servo={servo_pin} (PWM {self.servo.kind}) duty={self.duty:.0f}% durada={self.end_s:.1f}s. '
            'COTXE AIXECAT!'
        )
        self.start = time.monotonic()
        self.last_report = self.start
        self.last_ticks = 0
        self.create_timer(1.0 / PWM_FREQUENCY_HZ, self.update)

    # -------------------------
    # SERVO
    # -------------------------
    def _servo_pulse(self, t):
        """t a [-1, 1]: -1 = min_pulse_us, 0 = centre, 1 = max_pulse_us."""
        if self.servo_invert:
            t = -t
        if t >= 0.0:
            return self.center_pulse_us + t * (self.max_pulse_us - self.center_pulse_us)
        return self.center_pulse_us + t * (self.center_pulse_us - self.min_pulse_us)

    # -------------------------
    # MISSIÓ
    # -------------------------
    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.get_logger().info(f'Estat: {state}')

    def update(self):
        if self.finished:
            return
        t = time.monotonic() - self.start
        self.moving_pub.publish(Bool(data=self.ready_s <= t < self.ready_s + self.drive_s))

        if t < self.ready_s:
            self.set_state('PREPARAT')
            duty, steer = 0.0, 0.0
        elif t < self.ready_s + self.drive_s:
            self.set_state('INSPECCIÓ (motors + direcció en sinus)')
            t_drive = t - self.ready_s
            duty = self.duty * min(1.0, t_drive / self.ramp_s) if self.ramp_s > 0.0 else self.duty
            steer = self.steer_amplitude * math.sin(2.0 * math.pi * t_drive / self.steer_period_s)
        elif t < self.end_s:
            self.set_state('FRENADA (EBS)')
            for motor in self.motors:
                motor.brake()
            self.servo.set_pulse(self._servo_pulse(0.0))
            self.report(t)
            return
        else:
            self.report(t, force=True)
            self.set_state('ACABAT')
            self.finished = True
            return

        for motor in self.motors:
            motor.set(duty)
        if self.encoder is not None and duty != 0.0:
            self.encoder.direction = 1 if duty > 0.0 else -1
        self.servo.set_pulse(self._servo_pulse(steer))
        self.report(t)

    def report(self, t, force=False):
        now = time.monotonic()
        if self.encoder is None or (not force and now - self.last_report < 1.0):
            return
        ticks = self.encoder.ticks
        dt = max(now - self.last_report, 1e-3)
        rev_s = (ticks - self.last_ticks) / self.pulses_per_rev / dt
        speed = rev_s * math.pi * self.wheel_diameter
        self.get_logger().info(
            f'  t={t:4.1f}s ticks={ticks - self.last_ticks:+6d} (total {ticks:+d}) '
            f'~{rev_s * 60.0:+.0f} rpm roda, {speed:+.2f} m/s'
        )
        self.last_report = now
        self.last_ticks = ticks

    # -------------------------
    # CLEANUP
    # -------------------------
    def release_hardware(self):
        """Deixa motors i servo a 0 i allibera els pins (es pot cridar més d'un cop)."""
        if GPIO is None:
            return
        for motor in self.motors:
            try:
                motor.stop()
            except Exception:
                pass
        # La variable del bucle encara apunta a l'últim motor: s'esborra perquè
        # els seus PWM es destrueixin ara i no en sortir de Python, quan
        # rpi-lgpio ja no els pot aturar (TypeError).
        motor = None
        if self.servo is not None:
            self.servo.stop()
        self.motors = []
        self.servo = None
        # Motors i servo no es netegen (GPIO.cleanup()): es queden a 0 i, quan
        # el procés s'acaba, el pull-down de DRV8833Motor els manté a 0.
        # Només s'allibera l'encoder.
        if self.encoder is not None:
            try:
                GPIO.cleanup(list(self.encoder.pins))
            except Exception:
                pass
            self.encoder = None

    def destroy_node(self):
        self.release_hardware()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestMotors()
    deadline = time.monotonic() + MAX_TEST_S
    try:
        while rclpy.ok() and not node.finished:
            if time.monotonic() >= deadline:
                node.get_logger().warn(f'Límit de {MAX_TEST_S:.0f}s assolit: aturant motors')
                break
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
