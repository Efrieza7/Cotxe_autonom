import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('car_simulator')
    default_map = os.path.join(pkg_share, 'maps', 'sample_track.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'car_simulator.rviz')
    urdf_path = os.path.join(pkg_share, 'urdf', 'car.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    map_file = LaunchConfiguration('map_file')
    max_range = LaunchConfiguration('max_range')
    speed_mps = LaunchConfiguration('speed_mps')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map,
                               description='Path to the cone map YAML file'),
        DeclareLaunchArgument('max_range', default_value='2.0',
                               description='Max LiDAR detection range (m)'),
        DeclareLaunchArgument('speed_mps', default_value='0.0',
                               description='Initial speed (m/s), overridden as soon as '
                                            'codi_principal\'s path_follower publishes /target_speed'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='car_simulator',
            executable='car_simulator_node',
            name='car_simulator_node',
            output='screen',
            parameters=[{
                'wheelbase': 0.40,
                'track': 0.25,
                'speed_mps': speed_mps,
            }],
        ),

        Node(
            package='car_simulator',
            executable='lidar_simulator_node',
            name='lidar_simulator_node',
            output='screen',
            parameters=[{
                'map_file': map_file,
                'max_range': max_range,
                'rotation_frequency_hz': 10.0,
                'points_per_second': 4500.0,
            }],
        ),

        Node(
            package='car_simulator',
            executable='cone_map_publisher_node',
            name='cone_map_publisher_node',
            output='screen',
            parameters=[{'map_file': map_file}],
        ),

        Node(
            package='car_simulator',
            executable='path_visualizer_node',
            name='path_visualizer_node',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
            condition=IfCondition(use_rviz),
        ),
    ])
