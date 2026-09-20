"""Publishes the full ground-truth cone map as a MarkerArray for RViz.

Reloads the map file periodically, so switching tracks is just a matter of
editing/replacing the `map_file` parameter's target YAML file.
"""

from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .map_utils import get_mtime, load_cone_map


class ConeMapPublisherNode(Node):

    def __init__(self) -> None:
        super().__init__('cone_map_publisher_node')

        self.declare_parameter('map_file', '')
        self.declare_parameter('map_topic', '/simulator/ground_truth_map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('map_reload_period_sec', 2.0)

        self.map_file = str(self.get_parameter('map_file').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.cones = load_cone_map(self.map_file)
        self._map_mtime = get_mtime(self.map_file)
        self.get_logger().info(f'Loaded {len(self.cones)} cones from "{self.map_file}"')

        map_topic = str(self.get_parameter('map_topic').value)
        self.pub = self.create_publisher(MarkerArray, map_topic, 10)

        publish_period = float(self.get_parameter('publish_period_sec').value)
        self.create_timer(max(0.05, publish_period), self._publish_markers)

        reload_period = float(self.get_parameter('map_reload_period_sec').value)
        if reload_period > 0.0:
            self.create_timer(reload_period, self._reload_map_if_changed)

    def _reload_map_if_changed(self) -> None:
        mtime = get_mtime(self.map_file)
        if mtime != self._map_mtime:
            self._map_mtime = mtime
            self.cones = load_cone_map(self.map_file)
            self.get_logger().info(
                f'Map file changed, reloaded {len(self.cones)} cones from "{self.map_file}"'
            )

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, (x, y) in enumerate(self.cones):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = 'ground_truth_map'
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.065
            m.scale.y = 0.065
            m.scale.z = 0.2
            m.color.r = 1.0
            m.color.g = 0.6
            m.color.b = 0.0
            m.color.a = 0.9
            m.lifetime = Duration(sec=2, nanosec=0)
            marker_array.markers.append(m)

        self.pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConeMapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
