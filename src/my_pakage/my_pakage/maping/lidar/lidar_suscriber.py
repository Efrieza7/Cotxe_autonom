"""Simple LiDAR subscriber placed under nodes/lidar for organization."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_suscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10
        )
        self.get_logger().info('LiDAR subscriber started, listening on /ldlidar_node/scan')

    def listener_callback(self, msg: LaserScan):
        """Processa un missatge LaserScan i registra informació resumida.
        Mostra nombre total de punts, punts vàlids i la distància mínima vàlida.
        """
        ranges = msg.ranges
        # Filtrar NaN
        valid_ranges = [r for r in ranges if not math.isnan(r)]
        total = len(ranges)
        valid = len(valid_ranges)
        min_r = min(valid_ranges) if valid > 0 else float('nan')

        self.get_logger().info(
            'Scan received: total=%d valid=%d min=%.3f m' % (total, valid, min_r)
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
