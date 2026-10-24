import math

import rclpy
from rclpy.node import Node

from nodes.control.servo_pwm import PWM_FREQUENCY_HZ, ServoPwm


class TestServo(Node):
    """Node de prova del servo de la direcció.

    Fa oscil·lar el servo suaument a esquerra i dreta al voltant del centre
    durant uns quants cicles, el torna a deixar centrat i s'atura.

    Fa servir el mateix pin i les mateixes amplades de pols per defecte que
    `steering`, així que si allà funciona, aquí també.
    """

    def __init__(self):
        super().__init__('test_servo')

        self.servo_pin = int(self.declare_parameter('servo_pin', 18).value)  # BCM (GPIO 18 = pin físic 12)
        self.min_pulse_us = float(self.declare_parameter('min_pulse_us', 1000.0).value)
        self.center_pulse_us = float(self.declare_parameter('center_pulse_us', 1500.0).value)
        self.max_pulse_us = float(self.declare_parameter('max_pulse_us', 2000.0).value)
        # Fracció del recorregut total (0..1) que es mou a cada costat.
        self.amplitude = float(self.declare_parameter('amplitude', 0.3).value)
        self.period_s = float(self.declare_parameter('period_s', 2.0).value)
        self.cycles = int(self.declare_parameter('cycles', 3).value)

        self.amplitude = max(0.0, min(1.0, self.amplitude))
        self.elapsed = 0.0
        self.dt = 1.0 / PWM_FREQUENCY_HZ
        self.finished = False

        self.servo = None
        try:
            self.servo = ServoPwm(self.servo_pin, self._pulse_us(0.0), warn=self.get_logger().warning)
        except Exception as e:
            self.get_logger().error(
                f'No es pot generar el senyal del servo ({e}). '
                'No es mourà cap servo, només es mostren els polsos.'
            )

        self.get_logger().info(
            f'TestServo started: servo_pin={self.servo_pin} '
            f'(PWM {self.servo.kind if self.servo else "cap"}) amplitude={self.amplitude:.2f} '
            f'period={self.period_s:.1f}s cycles={self.cycles}'
        )
        self.timer = self.create_timer(self.dt, self.update)

    def _pulse_us(self, t: float) -> float:
        """t a [-1, 1]: -1 = min_pulse_us, 0 = centre, 1 = max_pulse_us."""
        if t >= 0.0:
            return self.center_pulse_us + t * (self.max_pulse_us - self.center_pulse_us)
        return self.center_pulse_us + t * (self.center_pulse_us - self.min_pulse_us)

    def update(self) -> None:
        self.elapsed += self.dt
        total = self.period_s * self.cycles

        if self.elapsed >= total:
            # Deixa el servo al centre i dona-li mig segon per arribar-hi.
            if self.servo is not None:
                self.servo.set_pulse(self._pulse_us(0.0))
            if self.elapsed >= total + 0.5:
                self.get_logger().info('Prova acabada: servo centrat')
                self.timer.cancel()
                self.finished = True
            return

        t = self.amplitude * math.sin(2.0 * math.pi * self.elapsed / self.period_s)
        if self.servo is not None:
            self.servo.set_pulse(self._pulse_us(t))
        self.get_logger().info(f'pols = {self._pulse_us(t):.0f} us', throttle_duration_sec=0.25)

    def destroy_node(self):
        if self.servo is not None:
            self.servo.set_pulse(self._pulse_us(0.0))
            self.servo.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestServo()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
