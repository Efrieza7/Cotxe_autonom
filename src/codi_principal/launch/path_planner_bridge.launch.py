from launch import LaunchDescription
from launch_ros.actions import Node


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
                        "max_steer_rad": 0.785398,
                        "target_speed": 0.8,
                    }
                ],
            ),
        ]
    )

