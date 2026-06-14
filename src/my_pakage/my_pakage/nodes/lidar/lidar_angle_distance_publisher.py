import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math


class LidarAngleDistancePublisher(Node):
    def __init__(self):
        super().__init__('lidar_angle_distance_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10
        )
        self.pub = self.create_publisher(Float32MultiArray, '/ldlidar_node/scan_angle_distance', 10)
        # publica a 20 Hz (0.05 s) independentment de la taxa d'arribada d'scans
        self._publish_period = 0.05
        self._last_out = None
        self._timer = self.create_timer(self._publish_period, self._timer_callback)
        self.get_logger().info('Angle-distance publisher started, listening on /ldlidar_node/scan; republishing at 20 Hz')

    def listener_callback(self, msg: LaserScan):
        """Convert LaserScan to a Float32MultiArray of interleaved [angle, distance] pairs.
        Angles are in radians.
        Only valid ranges (finite numbers within [range_min, range_max]) are included.
        """
        angles_and_distances = []
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and (r >= msg.range_min) and (r <= msg.range_max):
                angles_and_distances.append(float(angle))
                angles_and_distances.append(float(r))
            angle += msg.angle_increment

        # desa l'última matriu convertida; el temporitzador publicarà a la taxa sol·licitada
        out = Float32MultiArray()
        out.data = angles_and_distances
        self._last_out = out
        self.get_logger().debug(f'Prepared {len(angles_and_distances)//2} pairs from scan for republishing')

    def _timer_callback(self):
        if self._last_out is None or not self._last_out.data:
            return
        self.pub.publish(self._last_out)
        # el registre de debug a la taxa del temporitzador pot ser sorollós; mantén informació mínima
        self.get_logger().debug(f'Republished {len(self._last_out.data)//2} pairs')


def main(args=None):
    rclpy.init(args=args)
    node = LidarAngleDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
