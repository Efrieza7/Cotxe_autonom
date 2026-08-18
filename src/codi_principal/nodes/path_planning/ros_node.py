"""ROS 2 node: colorblind autocross path planner.

Subscribes to the cone map published by LidarProcessing and publishes
an ordered list of centerline waypoints suitable for downstream
controllers.

Topics
------
Subscribed:
    /ldlidar_node/cons_map  (my_pakage_msgs/ConsMap)
        Interleaved [x, y, count, x, y, count, ...] cone positions.

Published:
    /path_planning/waypoints  (std_msgs/Float32MultiArray)
        Interleaved [x0, y0, x1, y1, ...] centerline waypoints.
        Empty array when not enough cones are detected.

Parameters
----------
smooth_window : int, default 3
    Moving-average window size for centerline smoothing.
min_edge_length : float, default 0.3
    Minimum Delaunay edge length considered (metres).
max_edge_length : float, default 6.0
    Maximum Delaunay edge length considered (metres).
min_cone_count : int, default 2
    Minimum cone observation count to include a cone.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from my_pakage_msgs.msg import ConsMap

from .planner import compute_centerline


class PathPlanningNode(Node):
    """Colorblind centerline path planner node."""

    def __init__(self):
        super().__init__('path_planning_node')

        # Declare configurable parameters
        self.declare_parameter('smooth_window', 3)
        self.declare_parameter('min_edge_length', 0.3)
        self.declare_parameter('max_edge_length', 6.0)
        self.declare_parameter('min_cone_count', 2)

        self.subscription = self.create_subscription(
            ConsMap,
            '/ldlidar_node/cons_map',
            self._cons_map_callback,
            10,
        )

        self.pub_waypoints = self.create_publisher(
            Float32MultiArray,
            '/path_planning/waypoints',
            10,
        )

        self.get_logger().info(
            'PathPlanningNode started: subscribing /ldlidar_node/cons_map'
        )

    def _cons_map_callback(self, msg: ConsMap):
        try:
            data = list(msg.data or [])
            # Extract cone centres; filter by minimum observation count
            min_count = self.get_parameter('min_cone_count').value
            cones = []
            for i in range(0, len(data) - 2, 3):
                x, y, count = data[i], data[i + 1], data[i + 2]
                if count >= min_count:
                    cones.append((float(x), float(y)))

            smooth_window = self.get_parameter('smooth_window').value
            min_edge = self.get_parameter('min_edge_length').value
            max_edge = self.get_parameter('max_edge_length').value

            waypoints = compute_centerline(
                cones,
                smooth_window=int(smooth_window),
                min_edge_length=float(min_edge),
                max_edge_length=float(max_edge),
            )

            out = Float32MultiArray()
            out.data = [v for pt in waypoints for v in pt]
            self.pub_waypoints.publish(out)

            self.get_logger().info(
                f'PathPlanning: {len(cones)} cones → {len(waypoints)} waypoints'
            )
        except Exception as e:
            self.get_logger().error(f'PathPlanningNode error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
