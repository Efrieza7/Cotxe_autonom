#!/usr/bin/env python3
"""ROS 2 bridge between local cone/pose topics and fsd_path_planning."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from my_pakage_msgs.msg import ConsMap
from .bridge_utils import (
    build_unknown_cone_observations,
    extract_xy_path,
)

_FSD_PATH = Path(__file__).resolve().parent / "ft-fsd-path-planning-main"
if str(_FSD_PATH) not in sys.path:
    sys.path.insert(0, str(_FSD_PATH))

from fsd_path_planning import ConeTypes, MissionTypes, PathPlanner


class PathPlannerBridgeNode(Node):
    """Bridge node using unknown-color cones for fsd path planning."""

    def __init__(self) -> None:
        super().__init__("path_planner_bridge")

        self.declare_parameter("map_topic", "/ldlidar_node/cons_map")
        self.declare_parameter("pose_topic", "/pose")
        self.declare_parameter("path_topic", "/path_planning/waypoints")
        self.declare_parameter("mission", "trackdrive")
        self.declare_parameter("experimental_performance_improvements", False)
        self.declare_parameter("min_cone_count", 1)
        self.declare_parameter("timer_period_sec", 0.1)

        self.map_topic = self.get_parameter("map_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.min_cone_count = int(self.get_parameter("min_cone_count").value)
        timer_period = float(self.get_parameter("timer_period_sec").value)
        mission = self._mission_from_param(str(self.get_parameter("mission").value))
        experimental = bool(
            self.get_parameter("experimental_performance_improvements").value
        )

        self.planner = PathPlanner(
            mission=mission,
            experimental_performance_improvements=experimental,
        )

        self.pose_sub = self.create_subscription(
            Float32MultiArray, self.pose_topic, self.pose_callback, 10
        )
        self.cones_sub = self.create_subscription(
            ConsMap, self.map_topic, self.map_callback, 10
        )
        self.path_pub = self.create_publisher(Float32MultiArray, self.path_topic, 10)

        self.last_pose_xy: np.ndarray | None = None
        self.last_dir_xy: np.ndarray | None = None
        self.last_cones = None
        self.last_cones_count = 0
        self.has_new_inputs = False

        self.create_timer(max(0.02, timer_period), self._tick)
        self.get_logger().info(
            "Path planner bridge started. "
            f"cones={self.map_topic} pose={self.pose_topic} path={self.path_topic}"
        )

    def _mission_from_param(self, mission_str: str) -> MissionTypes:
        mission_map = {
            "trackdrive": MissionTypes.trackdrive,
            "autocross": MissionTypes.autocross,
            "acceleration": MissionTypes.acceleration,
            "skidpad": MissionTypes.skidpad,
            "ebs_test": MissionTypes.ebs_test,
        }
        mission = mission_map.get(mission_str.lower())
        if mission is None:
            self.get_logger().warning(
                f"Unknown mission '{mission_str}', defaulting to 'trackdrive'"
            )
            return MissionTypes.trackdrive
        return mission

    def pose_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warning(
                "Pose message must contain at least [x, y, yaw]; ignoring."
            )
            return

        try:
            x = float(msg.data[0])
            y = float(msg.data[1])
            yaw = float(msg.data[2])
        except (TypeError, ValueError):
            self.get_logger().warning("Malformed pose values; ignoring.")
            return

        if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(yaw):
            self.get_logger().warning("Pose contains non-finite values; ignoring.")
            return

        self.last_pose_xy = np.asarray([x, y], dtype=np.float64)
        self.last_dir_xy = np.asarray([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
        self.has_new_inputs = True

    def map_callback(self, msg: ConsMap) -> None:
        self.last_cones = build_unknown_cone_observations(
            cons_data=msg.data,
            cone_types_count=len(ConeTypes),
            unknown_index=int(ConeTypes.UNKNOWN),
            min_cone_count=self.min_cone_count,
        )
        self.last_cones_count = len(self.last_cones[int(ConeTypes.UNKNOWN)])
        self.has_new_inputs = True

    def _tick(self) -> None:
        if not self.has_new_inputs:
            return
        self.has_new_inputs = False

        if self.last_pose_xy is None or self.last_dir_xy is None:
            self.get_logger().warning("Waiting for pose input before planning.")
            return
        if self.last_cones is None:
            self.get_logger().warning("Waiting for cone map input before planning.")
            return

        if self.last_cones_count == 0:
            self.get_logger().warning("No usable cones in ConsMap; publishing empty path.")
            self.path_pub.publish(Float32MultiArray(data=[]))
            return

        try:
            planner_result = self.planner.calculate_path_in_global_frame(
                self.last_cones, self.last_pose_xy, self.last_dir_xy
            )
        except Exception as exc:
            self.get_logger().error(f"Path planner failed: {exc}")
            return

        path_xy = extract_xy_path(planner_result)
        if path_xy is None or len(path_xy) == 0:
            self.get_logger().warning("Planner returned malformed/empty path output.")
            self.path_pub.publish(Float32MultiArray(data=[]))
            return

        flattened = [float(v) for point in path_xy for v in point]
        self.path_pub.publish(Float32MultiArray(data=flattened))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlannerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
