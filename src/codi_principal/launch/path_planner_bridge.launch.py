import math

from launch import LaunchDescription
from launch_ros.actions import Node

# Angle màxim real de les rodes (rad): igual que full_car.launch.py,
# angle_rodes = 24.52 deg · sin(angle_servo) amb el servo a 45 deg (~17.3 deg).
MAX_STEER = math.radians(24.52) * math.sin(math.radians(45.0))


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="my_pakage",
                executable="path_planner_bridge",
                name="path_planner_bridge",
                output="screen",
                parameters=[
                    {
                        "map_topic": "/ldlidar_node/cons_map",
                        "pose_topic": "/pose",
                        "path_topic": "/path_planning/waypoints",
                        "mission": "trackdrive",
                        "experimental_performance_improvements": False,
                        "min_cone_count": 1,
                        "timer_period_sec": 0.1,
                        "planner_scale": 10.0,
                    }
                ],
            ),
            Node(
                package="my_pakage",
                executable="path_follower",
                name="path_follower",
                output="screen",
                parameters=[
                    {
                        "path_topic": "/path_planning/waypoints",
                        "pose_topic": "/pose",
                        "steering_topic": "target_angle",
                        "speed_topic": "target_speed",
                        "lookahead": 0.3,
                        "wheelbase": 0.18,
                        "max_steer_rad": MAX_STEER,
                        "target_speed": 0.8,
                    }
                ],
            ),
        ]
    )

