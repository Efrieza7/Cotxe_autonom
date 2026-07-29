import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from my_pakage_msgs.msg import ConsMap


class ConsMapViz(Node):
    """Subscribe to ConsMap and publish MarkerArray for RViz visualization."""

    def __init__(self):
        super().__init__('cons_map_viz')
        self.sub = self.create_subscription(
            ConsMap,
            '/ldlidar_node/cons_map',
            self.cons_cb,
            10,
        )
        self.pub = self.create_publisher(MarkerArray, '/cons_map_markers', 10)
        # Default frame used for markers; change if your TF uses a different frame
        self.frame_id = 'map'
        self.get_logger().info('ConsMapViz started: listening /ldlidar_node/cons_map')

    def cons_cb(self, msg: ConsMap):
        ma = MarkerArray()
        for i in range(0, len(msg.data), 3):
            try:
                x = float(msg.data[i])
                y = float(msg.data[i + 1])
                count = int(msg.data[i + 2])
            except Exception:
                continue

            m = Marker()
            m.header.frame_id = 'ldlidar_base'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'cons_map'
            m.id = i // 3
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            # scale marker by detected count (so larger clusters look bigger)
            size = 0.1
            m.scale.x = float(size)
            m.scale.y = float(size)
            m.scale.z = float(size)

            # color: red with a green component proportional to count
            m.color.r = 1.0
            m.color.g = min(1.0, count / 10.0)
            m.color.b = 0.0
            m.color.a = 0.9

            # short lifetime so markers refresh if map changes; set to 0 for persistent
            m.lifetime = Duration(sec=1, nanosec=0)

            ma.markers.append(m)

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ConsMapViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
