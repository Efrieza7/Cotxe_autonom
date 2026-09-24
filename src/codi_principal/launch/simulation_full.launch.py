import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Full simulation stack: car_simulator + mapping + path planning + control.

    This is the launch to use for testing the car in simulation end-to-end:
        car_simulator (car_simulator_node, lidar_simulator_node,
                       cone_map_publisher_node, rviz2)
      + mapping (lidar_image_creator, lidar_processing, cons_map_viz)
      + path_planner_bridge + path_follower (drives the simulated car by
        publishing /target_angle and /target_speed, which car_simulator_node
        consumes)
    """

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('track', default_value='bcn',
                              description='Circuit to load: ' + ', '.join(TRACKS)),
        OpaqueFunction(function=_launch_setup),
    ])


# Each track: map file in car_simulator/maps and a start pose (x, y, yaw) on its
# centreline, heading along the track. All maps are 0.35 m wide.
TRACKS = {
    'bcn': ('circuit_BCN.yaml', '1.384', '0.785', '-2.131'),
    'realista': ('circuit_realista_35.yaml', '0.0', '0.0', '1.5596'),
}


def _launch_setup(context):
    use_rviz = LaunchConfiguration('use_rviz')
    track = LaunchConfiguration('track').perform(context)
    if track not in TRACKS:
        raise RuntimeError(f"Unknown track '{track}'. Options: {', '.join(TRACKS)}")
    map_name, start_x, start_y, start_yaw = TRACKS[track]

    car_simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_simulator'),
                'launch',
                'car_simulator.launch.py',
            )
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'map_file': os.path.join(get_package_share_directory('car_simulator'), 'maps', map_name),
            'start_x': start_x,
            'start_y': start_y,
            'start_yaw': start_yaw,
        }.items(),
    )

    mapping_nodes = [
        Node(package='my_pakage', executable='lidar_image_creator', name='lidar_image_creator', output='screen'),
        Node(package='my_pakage', executable='lidar_processing', name='lidar_processing', output='screen'),
        Node(package='my_pakage', executable='cons_map_viz', name='cons_map_viz', output='screen'),
    ]

    control_nodes = [
        Node(
            package='my_pakage',
            executable='path_planner_bridge',
            name='path_planner_bridge',
            output='screen',
            parameters=[{
                'map_topic': '/ldlidar_node/cons_map',
                'pose_topic': '/pose',
                'path_topic': '/path_planning/waypoints',
                'mission': 'trackdrive',
                'experimental_performance_improvements': False,
                'min_cone_count': 1,
                'timer_period_sec': 0.1,
                'planner_scale': 10.0,
            }],
        ),
        Node(
            package='my_pakage',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[{
                'path_topic': '/path_planning/waypoints',
                'pose_topic': '/pose',
                'steering_topic': 'target_angle',
                'speed_topic': 'target_speed',
                'lookahead': 0.2,
                'wheelbase': 0.18,
                'max_steer_rad': 0.785398,
                'target_speed': 0.5,
            }],
        ),
    ]

    return [car_simulator_include, *mapping_nodes, *control_nodes]
