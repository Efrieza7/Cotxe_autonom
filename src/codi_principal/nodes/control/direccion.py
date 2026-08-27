import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Direccion(Node):
    """Node que converteix l'`angle objectiu` en una comanda per servo.

    - Subscriu el topic `target_angle` (Float32).
    - Aplica un limitador de velocitat (slew rate) per evitar salts bruscos.
    - Mapeja l'angle a una sortida segons el `mode` configurat:
      - 'degrees': publica graus (Float32)
      - 'normalized': publica [0..1]
      - 'pwm': publica microsegons PWM (Float32)
    - Publica a `servo_command` (Float32).
    """

    def __init__(self):
        super().__init__('direccion')

        # ROS interfaces
        self.sub = self.create_subscription(Float32, 'target_angle', self.callback_target, 10)
        self.sub = self.create_subscription(Float32, 'ldlidar_node/LaserScan', self.callback_lidar, 10)
        self.pub = self.create_publisher(Float32, 'servo_command', 10)

        # Parameters
        self.input_is_degrees = bool(self.declare_parameter('input_is_degrees', False).value)
        self.mode = str(self.declare_parameter('mode', 'degrees').value)  # degrees|normalized|pwm

        # Mapping params for pwm/normalized
        self.min_angle = float(self.declare_parameter('min_angle', -0.785398).value)  # rad
        self.max_angle = float(self.declare_parameter('max_angle', 0.785398).value)   # rad
        self.min_pwm = float(self.declare_parameter('min_pwm', 1000.0).value)
        self.max_pwm = float(self.declare_parameter('max_pwm', 2000.0).value)

        # Slew-rate limiting (degrees or radians per second depending on input)
        self.max_delta_per_sec = float(self.declare_parameter('max_delta_per_sec', 1.0).value)

        # Internal state
        self.last_output = None
        self.last_time = None

        self.get_logger().info(f'Direccion node started mode={self.mode} input_degrees={self.input_is_degrees}')

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def map_to_pwm(self, angle_rad: float) -> float:
        # map angle in radians (min_angle..max_angle) to pwm (min_pwm..max_pwm)
        a = self.clamp(angle_rad, self.min_angle, self.max_angle)
        t = (a - self.min_angle) / (self.max_angle - self.min_angle) if (self.max_angle - self.min_angle) != 0 else 0.5
        return self.min_pwm + t * (self.max_pwm - self.min_pwm)

    def map_to_normalized(self, angle_rad: float) -> float:
        a = self.clamp(angle_rad, self.min_angle, self.max_angle)
        return (a - self.min_angle) / (self.max_angle - self.min_angle) if (self.max_angle - self.min_angle) != 0 else 0.5
    def callback_target(self, msg: Float32) -> None:
        self.angle = float(msg.data)

    def callback_lidar(self, msg: Float32) -> None:
        # Read input angle
        angle = self.angle

        # Convert degrees->radians if needed
        if self.input_is_degrees:
            # degrees to radians
            angle = angle * 3.141592653589793 / 180.0

        now = self.get_clock().now()
        if self.last_time is None:
            dt = 0.02
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt <= 0:
                dt = 1e-3

        # Apply slew rate limit (angle units per second)
        if self.last_output is None:
            limited_angle = angle
        else:
            max_delta = self.max_delta_per_sec * dt
            delta = angle - self.last_output
            if delta > max_delta:
                limited_angle = self.last_output + max_delta
            elif delta < -max_delta:
                limited_angle = self.last_output - max_delta
            else:
                limited_angle = angle

        # Prepare output according to mode
        out = Float32()
        if self.mode == 'degrees':
            # publish degrees for servo controller
            deg = limited_angle * 180.0 / 3.141592653589793
            out.data = float(deg)
        elif self.mode == 'normalized':
            out.data = float(self.map_to_normalized(limited_angle))
        elif self.mode == 'pwm':
            out.data = float(self.map_to_pwm(limited_angle))
        else:
            # fallback: publish radians
            out.data = float(limited_angle)

        # Publish and update state
        self.pub.publish(out)
        self.last_output = limited_angle
        self.last_time = now


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
