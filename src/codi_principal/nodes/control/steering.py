import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    GPIO = None


class Steering(Node):
    """Mou el servo de la direcció a partir de l'angle objectiu.

    - Subscriu `target_angle` (Float32, radians, positiu = gir a l'esquerra),
      publicat per `path_follower`. És l'angle de les rodes, no el del servo.
    - Converteix l'angle de les rodes en angle del servo amb la transmissió
      Ackermann-servomotor: angle_rodes = K · sin(angle_servo), on K és
      `wheel_angle_gain_deg` (el màxim teòric, amb el servo a 90°).
    - Aplica un limitador de velocitat (slew rate) per evitar salts bruscos.
    - Genera el PWM del servo directament des d'un GPIO de la Raspberry
      (50 Hz, amplada de pols entre `min_pulse_us` i `max_pulse_us`).
    - Publica a `steering_angle` (Float32, radians) l'angle de les rodes
      realment ordenat, que fa servir `bicycle_model` per a la localització.

    Si RPi.GPIO no està disponible o `dry_run` és True, no mou cap servo però
    continua publicant `steering_angle` (útil per provar-ho fora del cotxe).
    """

    PWM_FREQUENCY_HZ = 50.0

    def __init__(self):
        super().__init__('steering')

        self.servo_pin = int(self.declare_parameter('servo_pin', 4).value)  # BCM (GPIO 4 = pin físic 7)
        # Angle del servo (rad) a min_pulse_us / max_pulse_us.
        self.servo_max_angle = float(self.declare_parameter('servo_max_angle', 0.785398).value)
        # Transmissió servo -> rodes: angle_rodes = K · sin(angle_servo), K en graus.
        gain_deg = float(self.declare_parameter('wheel_angle_gain_deg', 24.52).value)
        self.wheel_gain = math.radians(gain_deg)
        # Angle màxim de les rodes que permet el recorregut del servo (sin és màxim a 90°).
        self.max_angle = self.wheel_gain * math.sin(min(self.servo_max_angle, math.pi / 2))
        # Amplada de pols per a -servo_max_angle, 0 i +servo_max_angle (microsegons).
        self.min_pulse_us = float(self.declare_parameter('min_pulse_us', 1000.0).value)
        self.center_pulse_us = float(self.declare_parameter('center_pulse_us', 1500.0).value)
        self.max_pulse_us = float(self.declare_parameter('max_pulse_us', 2000.0).value)
        # True si el servo gira al revés (pols més llarg = dreta).
        self.invert = bool(self.declare_parameter('invert', False).value)
        self.max_delta_per_sec = float(self.declare_parameter('max_delta_per_sec', 6.0).value)  # rad/s
        self.update_rate_hz = float(self.declare_parameter('update_rate_hz', 50.0).value)
        dry_run = bool(self.declare_parameter('dry_run', False).value)

        self.target_angle = 0.0
        self.current_angle = 0.0
        self.last_time = self.get_clock().now()

        self.pwm = None
        if dry_run or GPIO is None:
            self.get_logger().warning(
                'Steering en mode dry_run (sense RPi.GPIO o dry_run=True): '
                'no es mou cap servo, només es publica steering_angle'
            )
        else:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.servo_pin, GPIO.OUT)
            self.pwm = GPIO.PWM(self.servo_pin, self.PWM_FREQUENCY_HZ)
            self.pwm.start(self._duty_cycle(0.0))

        self.create_subscription(Float32, 'target_angle', self.callback_target, 10)
        self.pub = self.create_publisher(Float32, 'steering_angle', 10)
        self.create_timer(1.0 / max(1.0, self.update_rate_hz), self.update)

        self.get_logger().info(
            f'Steering started: servo_pin={self.servo_pin} '
            f'pulses={self.min_pulse_us}/{self.center_pulse_us}/{self.max_pulse_us} us '
            f'servo_max={math.degrees(self.servo_max_angle):.0f} deg '
            f'wheel_max={math.degrees(self.max_angle):.1f} deg (K={gain_deg} deg)'
        )

    def callback_target(self, msg: Float32) -> None:
        if math.isfinite(msg.data):
            self.target_angle = max(-self.max_angle, min(self.max_angle, float(msg.data)))

    def _servo_angle(self, wheel_angle: float) -> float:
        """Inversa de angle_rodes = K · sin(angle_servo)."""
        if self.wheel_gain <= 0.0:
            return 0.0
        return math.asin(max(-1.0, min(1.0, wheel_angle / self.wheel_gain)))

    def _pulse_us(self, wheel_angle: float) -> float:
        servo = self._servo_angle(wheel_angle)
        t = servo / self.servo_max_angle if self.servo_max_angle > 0.0 else 0.0
        t = max(-1.0, min(1.0, t))
        if self.invert:
            t = -t
        if t >= 0.0:
            return self.center_pulse_us + t * (self.max_pulse_us - self.center_pulse_us)
        return self.center_pulse_us + t * (self.center_pulse_us - self.min_pulse_us)

    def _duty_cycle(self, angle: float) -> float:
        period_us = 1e6 / self.PWM_FREQUENCY_HZ
        return 100.0 * self._pulse_us(angle) / period_us

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        max_delta = self.max_delta_per_sec * max(dt, 0.0)
        delta = self.target_angle - self.current_angle
        self.current_angle += max(-max_delta, min(max_delta, delta))

        if self.pwm is not None:
            self.pwm.ChangeDutyCycle(self._duty_cycle(self.current_angle))

        self.pub.publish(Float32(data=float(self.current_angle)))

    def destroy_node(self):
        if self.pwm is not None:
            self.pwm.ChangeDutyCycle(self._duty_cycle(0.0))
            self.pwm.stop()
            GPIO.cleanup(self.servo_pin)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Steering()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
