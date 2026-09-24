import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    GPIO = None


class Direccion(Node):
    """Mou el servo de la direcció a partir de l'angle objectiu.

    - Subscriu `target_angle` (Float32, radians, positiu = gir a l'esquerra),
      publicat per `path_follower`.
    - Aplica un limitador de velocitat (slew rate) per evitar salts bruscos.
    - Genera el PWM del servo directament des d'un GPIO de la Raspberry
      (50 Hz, amplada de pols entre `min_pulse_us` i `max_pulse_us`).
    - Publica a `steering_angle` (Float32, radians) l'angle realment ordenat,
      que fa servir `bycicle_mode` per a la localització.

    Si RPi.GPIO no està disponible o `dry_run` és True, no mou cap servo però
    continua publicant `steering_angle` (útil per provar-ho fora del cotxe).
    """

    PWM_FREQUENCY_HZ = 50.0

    def __init__(self):
        super().__init__('direccion')

        self.servo_pin = int(self.declare_parameter('servo_pin', 13).value)  # BCM
        self.max_angle = float(self.declare_parameter('max_angle', 0.785398).value)  # rad
        # Amplada de pols per a -max_angle, 0 i +max_angle (microsegons).
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
                'Direccion en mode dry_run (sense RPi.GPIO o dry_run=True): '
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
            f'Direccion started: servo_pin={self.servo_pin} '
            f'pulses={self.min_pulse_us}/{self.center_pulse_us}/{self.max_pulse_us} us '
            f'max_angle={math.degrees(self.max_angle):.0f} deg'
        )

    def callback_target(self, msg: Float32) -> None:
        if math.isfinite(msg.data):
            self.target_angle = max(-self.max_angle, min(self.max_angle, float(msg.data)))

    def _pulse_us(self, angle: float) -> float:
        t = angle / self.max_angle if self.max_angle > 0.0 else 0.0
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
    node = Direccion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
