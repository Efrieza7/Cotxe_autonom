"""Simulated 2D LiDAR that only reports cones within a configurable range.

Mimics an LD19-class LiDAR: `points_per_second` point rate (~4500 Hz) and
`rotation_frequency_hz` rotations per second (~10 Hz), so a full scan carries
`points_per_second / rotation_frequency_hz` beams. Publishes a real
`sensor_msgs/LaserScan` on `/ldlidar_node/scan`, so the existing
`ldlidar_listener` / `lidar_image_creator` / `lidar_processing` pipeline
(which already performs the pose correction) runs unmodified.

Each beam is computed from the car's own instantaneous pose at the moment
it was captured (not a single snapshot for the whole scan): at speed, the
car moves between the first and last beam of a rotation, exactly like a
real LiDAR would. `lidar_image_creator` undoes this same displacement using
`pose_at_offset`, so the raw scan must contain it or the correction shifts
points that were never actually offset.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from .map_utils import get_mtime, load_cone_map


class LidarSimulatorNode(Node):

    def __init__(self) -> None:
        super().__init__('lidar_simulator_node')

        self.declare_parameter('map_file', '')
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('cone_radius', 0.0325)
        self.declare_parameter('rotation_frequency_hz', 10.0)
        self.declare_parameter('points_per_second', 4500.0)
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('wheelbase', 0.40)
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('frame_id', 'ldlidar_base')
        self.declare_parameter('map_reload_period_sec', 2.0)

        self.map_file = str(self.get_parameter('map_file').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.cone_radius = float(self.get_parameter('cone_radius').value)
        self.rotation_frequency = float(self.get_parameter('rotation_frequency_hz').value)
        self.points_per_second = float(self.get_parameter('points_per_second').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.points_per_scan = max(8, int(round(self.points_per_second / self.rotation_frequency)))
        self.angle_increment = 2.0 * math.pi / self.points_per_scan
        self.time_increment = 1.0 / self.points_per_second
        self.scan_time = 1.0 / self.rotation_frequency

        self.all_cones: List[Tuple[float, float]] = load_cone_map(self.map_file)
        self._map_mtime = get_mtime(self.map_file)
        self.get_logger().info(
            f'Loaded {len(self.all_cones)} cones from "{self.map_file}"'
        )

        self.pose = (0.0, 0.0, 0.0, 0.0, 0.0)  # x, y, yaw, v, steer

        pose_topic = str(self.get_parameter('pose_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        self.create_subscription(Float32MultiArray, pose_topic, self._pose_cb, 10)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)

        self.create_timer(1.0 / self.rotation_frequency, self._publish_scan)

        reload_period = float(self.get_parameter('map_reload_period_sec').value)
        if reload_period > 0.0:
            self.create_timer(reload_period, self._reload_map_if_changed)

    def _reload_map_if_changed(self) -> None:
        mtime = get_mtime(self.map_file)
        if mtime != self._map_mtime:
            self._map_mtime = mtime
            self.all_cones = load_cone_map(self.map_file)
            self.get_logger().info(
                f'Map file changed, reloaded {len(self.all_cones)} cones from "{self.map_file}"'
            )

    def _pose_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 5:
            return
        self.pose = (
            float(msg.data[0]),
            float(msg.data[1]),
            float(msg.data[2]),
            float(msg.data[3]),
            float(msg.data[4]),
        )

    def _pose_at_offset(self, delta_t: float) -> Tuple[float, float, float]:
        """Mirror lidar_image_creator.pose_at_offset so beams carry the same displacement."""
        x, y, yaw, v, steer = self.pose
        yaw_rate = 0.0
        if abs(self.wheelbase) > 1e-6:
            yaw_rate = (v / self.wheelbase) * math.tan(steer)
        heading = yaw + yaw_rate * delta_t
        px = x + v * delta_t * math.cos(heading)
        py = y + v * delta_t * math.sin(heading)
        return px, py, heading

    def _cones_local_at(self, car_x: float, car_y: float, yaw: float) -> List[Tuple[float, float]]:
        """Return cones within `max_range` of (car_x, car_y), in that pose's local frame."""
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        visible = []
        for gx, gy in self.all_cones:
            dx = gx - car_x
            dy = gy - car_y
            if math.hypot(dx, dy) > self.max_range:
                continue
            # rotate into the car's local frame (inverse of yaw rotation)
            local_x = dx * cos_y + dy * sin_y
            local_y = -dx * sin_y + dy * cos_y
            visible.append((local_x, local_y))
        return visible

    def _ray_circle_hit(self, angle: float, cx: float, cy: float) -> float:
        """Return the distance to the near intersection of a ray with a cone circle, or inf."""
        dir_x = math.cos(angle)
        dir_y = math.sin(angle)
        proj = cx * dir_x + cy * dir_y
        if proj < 0.0:
            return math.inf
        perp2 = (cx * cx + cy * cy) - proj * proj
        r2 = self.cone_radius * self.cone_radius
        if perp2 > r2:
            return math.inf
        thc = math.sqrt(max(0.0, r2 - perp2))
        hit = proj - thc
        return hit if hit >= 0.0 else math.inf

    def _publish_scan(self) -> None:
        # msg.data at the end of the scan is treated as "now" (delta_t=0), matching
        # lidar_image_creator's (i - last_index) * time_increment convention.
        last_index = self.points_per_scan - 1

        ranges = [math.inf] * self.points_per_scan
        angle = -math.pi
        for i in range(self.points_per_scan):
            delta_t = (i - last_index) * self.time_increment
            beam_x, beam_y, beam_yaw = self._pose_at_offset(delta_t)
            cones_local = self._cones_local_at(beam_x, beam_y, beam_yaw)

            best = math.inf
            for cx, cy in cones_local:
                hit = self._ray_circle_hit(angle, cx, cy)
                if hit < best:
                    best = hit
            if self.range_min <= best <= self.max_range:
                ranges[i] = best
            angle += self.angle_increment

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.time_increment = self.time_increment
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.max_range
        scan.ranges = ranges

        self.scan_pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
