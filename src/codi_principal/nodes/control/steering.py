import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from nodes.control.servo_pwm import ServoPwm


class Steering(Node):
    """Mou el servo de la direcció a partir de l'angle objectiu.

    - Subscriu `target_angle` (Float32, radians, positiu = gir a l'esquerra),
      publicat per `path_follower`. És l'angle de les rodes, no el del servo.
    - Converteix l'angle de les rodes en angle del servo amb la transmissió
      de barres: el braç del servo (`servo_arm_mm`) i el de la mangueta
      (`knuckle_arm_mm`) desplacen la barra de direcció el mateix,
      servo_arm · sin(angle_servo) = knuckle_arm · sin(angle_rodes), és a dir,
      angle_rodes = asin(servo_arm / knuckle_arm · sin(angle_servo)).
      L'angle del servo surt del pols: 1000 us = 90°.
    - Suavitza l'ordre per evitar canvis bruscos: filtre passa-baix
      (`smoothing_time_sec`), zona morta per ignorar correccions petites
      (`deadband_rad`) i límit de velocitat de gir (`max_delta_per_sec`).
    - Genera el senyal del servo (50 Hz, amplada de pols entre `min_pulse_us`
      i `max_pulse_us`) amb `ServoPwm`: PWM de maquinari al GPIO 18.
    - Publica a `steering_angle` (Float32, radians) l'angle de les rodes
      realment ordenat, que fa servir `bicycle_model` per a la localització.

    Si no es pot generar el senyal o `dry_run` és True, no mou cap servo però
    continua publicant `steering_angle` (útil per provar-ho fora del cotxe).
    """

    def __init__(self):
        super().__init__('steering')

        self.servo_pin = int(self.declare_parameter('servo_pin', 18).value)  # BCM (GPIO 18 = pin físic 12)
        # Angle del servo (rad) a min_pulse_us / max_pulse_us.
        self.servo_max_angle = float(self.declare_parameter('servo_max_angle', 0.785398).value)
        # Transmissió servo -> rodes: angle_rodes = asin(servo_arm / knuckle_arm · sin(angle_servo)).
        self.servo_arm = float(self.declare_parameter('servo_arm_mm', 25.85).value)
        self.knuckle_arm = float(self.declare_parameter('knuckle_arm_mm', 22.0).value)
        if self.servo_arm <= 0.0 or self.knuckle_arm <= 0.0:
            raise ValueError('servo_arm_mm i knuckle_arm_mm han de ser > 0')
        self.arm_ratio = self.servo_arm / self.knuckle_arm
        # Angle màxim de les rodes que permet el recorregut del servo (sin és màxim a 90°).
        self.max_angle = self._wheel_angle(min(self.servo_max_angle, math.pi / 2))
        # Amplada de pols per a -servo_max_angle, 0 i +servo_max_angle (microsegons).
        self.min_pulse_us = float(self.declare_parameter('min_pulse_us', 1000.0).value)
        self.center_pulse_us = float(self.declare_parameter('center_pulse_us', 1500.0).value)
        self.max_pulse_us = float(self.declare_parameter('max_pulse_us', 2000.0).value)
        # True si el servo gira al revés (pols més llarg = dreta).
        self.invert = bool(self.declare_parameter('invert', False).value)
        # Velocitat màxima de gir de les rodes (rad/s): 1.0 = d'un extrem a l'altre (~0.6 rad) en ~0.6 s.
        self.max_delta_per_sec = float(self.declare_parameter('max_delta_per_sec', 1.0).value)
        # Constant de temps del filtre passa-baix de l'ordre (s). 0 = sense filtre.
        self.smoothing_time = float(self.declare_parameter('smoothing_time_sec', 0.3).value)
        # Correccions més petites que això (rad, ~0.6 deg) no mouen el servo.
        self.deadband = float(self.declare_parameter('deadband_rad', 0.01).value)
        self.update_rate_hz = float(self.declare_parameter('update_rate_hz', 50.0).value)
        dry_run = bool(self.declare_parameter('dry_run', False).value)

        self.target_angle = 0.0
        self.filtered_angle = 0.0
        self.current_angle = 0.0
        self.last_time = self.get_clock().now()

        self.servo = None
        if not dry_run:
            try:
                self.servo = ServoPwm(self.servo_pin, self._pulse_us(0.0), warn=self.get_logger().warning)
            except Exception as e:
                self.get_logger().error(f'No es pot generar el senyal del servo: {e}')
        if self.servo is None:
            self.get_logger().warning(
                'Steering en mode dry_run: no es mou cap servo, només es publica steering_angle'
            )

        self.create_subscription(Float32, 'target_angle', self.callback_target, 10)
        self.pub = self.create_publisher(Float32, 'steering_angle', 10)
        self.create_timer(1.0 / max(1.0, self.update_rate_hz), self.update)

        self.get_logger().info(
            f'Steering started: servo_pin={self.servo_pin} '
            f'(PWM {self.servo.kind if self.servo else "cap"}) '
            f'pulses={self.min_pulse_us}/{self.center_pulse_us}/{self.max_pulse_us} us '
            f'servo_max={math.degrees(self.servo_max_angle):.0f} deg '
            f'wheel_max={math.degrees(self.max_angle):.1f} deg '
            f'(braços servo/mangueta {self.servo_arm}/{self.knuckle_arm} mm)'
        )

    def callback_target(self, msg: Float32) -> None:
        if math.isfinite(msg.data):
            self.target_angle = max(-self.max_angle, min(self.max_angle, float(msg.data)))

    def _wheel_angle(self, servo_angle: float) -> float:
        """angle_rodes = asin(servo_arm / knuckle_arm · sin(angle_servo))."""
        return math.asin(max(-1.0, min(1.0, self.arm_ratio * math.sin(servo_angle))))

    def _servo_angle(self, wheel_angle: float) -> float:
        """Inversa: angle_servo = asin(knuckle_arm / servo_arm · sin(angle_rodes))."""
        return math.asin(max(-1.0, min(1.0, math.sin(wheel_angle) / self.arm_ratio)))

    def _pulse_us(self, wheel_angle: float) -> float:
        servo = self._servo_angle(wheel_angle)
        t = servo / self.servo_max_angle if self.servo_max_angle > 0.0 else 0.0
        t = max(-1.0, min(1.0, t))
        if self.invert:
            t = -t
        if t >= 0.0:
            return self.center_pulse_us + t * (self.max_pulse_us - self.center_pulse_us)
        return self.center_pulse_us + t * (self.center_pulse_us - self.min_pulse_us)

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        dt = max(dt, 0.0)
        if self.smoothing_time > 0.0:
            self.filtered_angle += (self.target_angle - self.filtered_angle) * min(1.0, dt / self.smoothing_time)
        else:
            self.filtered_angle = self.target_angle

        delta = self.filtered_angle - self.current_angle
        if abs(delta) > self.deadband:
            max_delta = self.max_delta_per_sec * dt
            self.current_angle += max(-max_delta, min(max_delta, delta))

        if self.servo is not None:
            self.servo.set_pulse(self._pulse_us(self.current_angle))

        self.pub.publish(Float32(data=float(self.current_angle)))

    def destroy_node(self):
        if self.servo is not None:
            self.servo.set_pulse(self._pulse_us(0.0))
            self.servo.stop()
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
