"""Visualizes the planned path (`/path_planning/waypoints`) in RViz."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker


class PathVisualizerNode(Node):

    def __init__(self) -> None:
        super().__init__('path_visualizer_node')

        self.declare_parameter('waypoints_topic', '/path_planning/waypoints')
        self.declare_parameter('path_topic', '/simulator/path')
        self.declare_parameter('markers_topic', '/simulator/path_markers')
        self.declare_parameter('frame_id', 'map')

        self.frame_id = str(self.get_parameter('frame_id').value)

        waypoints_topic = str(self.get_parameter('waypoints_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        markers_topic = str(self.get_parameter('markers_topic').value)

        self.create_subscription(Float32MultiArray, waypoints_topic, self._waypoints_cb, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.marker_pub = self.create_publisher(Marker, markers_topic, 10)

    def _waypoints_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data or [])
        usable_len = len(data) - (len(data) % 2)
        points = [(data[i], data[i + 1]) for i in range(0, usable_len, 2)]

        stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'planned_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in points]
        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
