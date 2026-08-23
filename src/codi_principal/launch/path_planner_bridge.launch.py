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
                    }
                ],
            )
        ]
    )
