import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    lidar_model = LaunchConfiguration('lidar_model')

    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ldlidar_node'),
                'launch',
                'ldlidar_with_mgr.launch.py'
            )
        ),
        launch_arguments={
            'serial_port': serial_port,
            'lidar_model': lidar_model,
        }.items()
    )

    lidar_listener_node = Node(
        package='my_pakage',
        executable='ldlidar_listener',
        name='ldlidar_listener',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ldlidar',
            description='Serial port for the lidar device'
        ),
        DeclareLaunchArgument(
            'lidar_model',
            default_value='LDLiDAR_LD19',
            description='Lidar model to use in the driver'
        ),
        ldlidar_launch,
        lidar_listener_node,
    ])
