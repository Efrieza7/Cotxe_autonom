import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from rclpy.time import Time


class ProximitiDireccion(Node):
    """Node que aplica un PID per generar un angle objectiu basat en sensors de proximitat.

    Rep un `Float32MultiArray` amb dos valors: [proximity, opposite].
    L'objectiu del PID és fer que la diferència entre els dos valors sigui 0.
    El node publica l'angle objectiu en `target_angle` com a `Float32`.
    """

    def __init__(self):
        super().__init__('proximiti_direccion')

        # ROS interfaces
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'proximity_values',
            self.listener_callback,
            10,
        )
        self.publisher = self.create_publisher(Float32, 'target_angle', 10)

        # PID parameters (valors predeterminats)
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.01

        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

        # Angle state and limits (radians)
        self.current_angle = 0.0
        self.min_angle = -0.785398  # -45 deg
        self.max_angle = 0.785398   # 45 deg

        self.get_logger().info('ProximitiDireccion node started (publishes target_angle)')

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def listener_callback(self, msg: Float32MultiArray):
        # Validate message
        if not msg.data or len(msg.data) < 2:
            self.get_logger().warning('Missatge incorrecte: calen dos valors')
            return

        proximity = float(msg.data[0])
        opposite = float(msg.data[1])

        # Error: volem que proximity - opposite == 0
        error = proximity - opposite

        # Time delta
        now = self.get_clock().now()
        if self.last_time is None:
            dt = 0.1
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt <= 0:
                dt = 1e-3

        # PID integration and derivative
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # PID output -> interpretarem control com a angle directe per ara
        control = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Assigna l'angle objectiu derivat del control i aplica límits
        self.current_angle = self.clamp(control, self.min_angle, self.max_angle)

        # Publica l'angle objectiu
        out_msg = Float32()
        out_msg.data = float(self.current_angle)
        self.publisher.publish(out_msg)

        # Log breu per seguiment
        self.get_logger().debug(
            f'proximity={proximity:.3f} opposite={opposite:.3f} error={error:.3f} angle={self.current_angle:.3f}'
        )

        # Update PID state
        self.prev_error = error
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ProximitiDireccion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
