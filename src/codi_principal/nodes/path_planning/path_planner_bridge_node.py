#!/usr/bin/env python3
"""ROS 2 bridge between local cone/pose topics and fsd_path_planning."""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from my_pakage_msgs.msg import ConsMap
from .bridge_utils import (
    build_cone_observations,
    extract_xy_path,
)

_FSD_PATH = Path(__file__).resolve().parent / "ft-fsd-path-planning-main"
if str(_FSD_PATH) not in sys.path:
    sys.path.insert(0, str(_FSD_PATH))


def _add_workspace_venv_site_packages() -> None:
    current_file = Path(__file__).resolve()
    for parent in current_file.parents:
        for site_packages in parent.glob('.venv/lib/python*/site-packages'):
            site_packages_str = str(site_packages)
            if site_packages_str not in sys.path:
                sys.path.insert(0, site_packages_str)


_add_workspace_venv_site_packages()

# fsd_path_planning JIT-compiles its numba functions on the first planner call
# (~30 s). By default numba caches them next to the installed sources, which is
# lost whenever install/ is wiped; keep the cache in the user's home instead so
# only the very first run pays that cost.
os.environ.setdefault(
    "NUMBA_CACHE_DIR",
    str(Path.home() / ".cache" / "cotxe_autonom" / "numba"),
)

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
        # fsd_path_planning is tuned for Formula Student scale (3 m wide track,
        # ~5 m between cones, 2.1 m car, several hard-coded metre constants).
        # Our track is 0.35 m wide with cones ~0.18 m apart, so cones and pose
        # are multiplied by this factor before planning and the path divided back.
        self.declare_parameter("planner_scale", 10.0)

        self.map_topic = self.get_parameter("map_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.min_cone_count = int(self.get_parameter("min_cone_count").value)
        timer_period = float(self.get_parameter("timer_period_sec").value)
        self.planner_scale = float(self.get_parameter("planner_scale").value)
        if not math.isfinite(self.planner_scale) or self.planner_scale <= 0.0:
            self.get_logger().warning("planner_scale must be > 0; using 1.0")
            self.planner_scale = 1.0
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
        self.has_new_inputs = False
        self.first_plan_done = False

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
        self.last_cones = build_cone_observations(
            cons_data=msg.data,
            cone_types_count=len(ConeTypes),
            unknown_index=int(ConeTypes.UNKNOWN),
            min_cone_count=self.min_cone_count,
            vehicle_position=self.last_pose_xy,
            vehicle_direction=self.last_dir_xy,
            left_index=int(ConeTypes.LEFT),
            right_index=int(ConeTypes.RIGHT),
        )
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

        if not any(len(cones) for cones in self.last_cones):
            self.get_logger().warning("No usable cones in ConsMap; publishing empty path.")
            self.path_pub.publish(Float32MultiArray(data=[]))
            return

        if not self.first_plan_done:
            self.get_logger().info(
                "First planner call: if numba has no cache yet it compiles "
                "now (~30 s); the car will start moving afterwards."
            )
        start = time.monotonic()
        try:
            scale = self.planner_scale
            planner_result = self.planner.calculate_path_in_global_frame(
                [cones * scale for cones in self.last_cones],
                self.last_pose_xy * scale,
                self.last_dir_xy,
            )
        except Exception as exc:
            self.get_logger().error(f"Path planner failed: {exc}")
            return

        if not self.first_plan_done:
            self.first_plan_done = True
            self.get_logger().info(
                f"First path computed in {time.monotonic() - start:.1f} s"
            )

        path_xy = extract_xy_path(planner_result)
        if path_xy is None or len(path_xy) == 0:
            self.get_logger().warning("Planner returned malformed/empty path output.")
            self.path_pub.publish(Float32MultiArray(data=[]))
            return

        path_xy = path_xy / self.planner_scale
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
