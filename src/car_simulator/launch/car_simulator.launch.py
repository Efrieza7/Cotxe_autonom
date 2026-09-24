import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Real car dimensions (m): must match codi_principal (path_follower, bycicle_mode,
# lidar_image_creator) and urdf/car.urdf.
WHEELBASE = 0.18
TRACK = 0.13
# Height of the LiDAR scan plane above the ground (m), same as urdf/car.urdf.
LIDAR_HEIGHT = 0.085


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
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map,
                               description='Path to the cone map YAML file'),
        DeclareLaunchArgument('max_range', default_value='2.0',
                               description='Max LiDAR detection range (m)'),
        DeclareLaunchArgument('speed_mps', default_value='0.0',
                               description='Initial speed (m/s), overridden as soon as '
                                            'codi_principal\'s path_follower publishes /target_speed'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_x', default_value='0.0',
                               description='Initial x (m), must be inside the track of map_file'),
        DeclareLaunchArgument('start_y', default_value='3.0',
                               description='Initial y (m), must be inside the track of map_file'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                               description='Initial heading (rad)'),

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
                'wheelbase': WHEELBASE,
                'track': TRACK,
                'speed_mps': speed_mps,
                'start_x': ParameterValue(start_x, value_type=float),
                'start_y': ParameterValue(start_y, value_type=float),
                'start_yaw': ParameterValue(start_yaw, value_type=float),
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
                'wheelbase': WHEELBASE,
                'lidar_height': LIDAR_HEIGHT,
                # same as the real driver: 455 bins per revolution at 10 Hz
                'rotation_frequency_hz': 10.0,
                'points_per_second': 4550.0,
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
