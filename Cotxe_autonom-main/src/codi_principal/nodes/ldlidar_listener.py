import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LDLidarListener(Node):
    def __init__(self):
        super().__init__('ldlidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.scan_callback,
            10,
        )
        self.subscription
        self.get_logger().info('LDLidar listener started on /ldlidar_node/scan')

    def scan_callback(self, msg: LaserScan) -> None:
        valid_ranges = [r for r in msg.ranges if r > 0.0]
        if not valid_ranges:
            self.get_logger().debug('Received LaserScan with no valid ranges yet')
            return

        closest = min(valid_ranges)
        if closest < 0.5:
            self.get_logger().info(f'Obstacle very near: {closest:.2f} m')
        else:
            self.get_logger().debug(f'Closest obstacle: {closest:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = LDLidarListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
