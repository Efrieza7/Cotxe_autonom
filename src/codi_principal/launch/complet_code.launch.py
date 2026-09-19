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

	# Include the existing lidar integration launch
	ldlidar_include = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('my_pakage'),
				'launch',
				'ldlidar_integration.launch.py',
			)
		),
		launch_arguments={'serial_port': serial_port, 'lidar_model': lidar_model}.items(),
	)

	nodes = [
		Node(package='my_pakage', executable='direccion', name='direccion', output='screen'),
		Node(package='my_pakage', executable='path_follower', name='path_follower', output='screen'),
		Node(package='my_pakage', executable='speed_control', name='speed_control', output='screen'),
		Node(package='my_pakage', executable='bycicle_mode', name='bycicle_mode', output='screen'),
		Node(package='my_pakage', executable='motor_reader', name='motor_reader', output='screen'),
		Node(package='my_pakage', executable='imu_suscriber', name='imu_suscriber', output='screen'),
		Node(package='my_pakage', executable='lidar_image_creator', name='lidar_image_creator', output='screen'),
		Node(package='my_pakage', executable='lidar_processing', name='lidar_processing', output='screen'),
		Node(package='my_pakage', executable='path_planner_bridge', name='path_planner_bridge', output='screen'),
	]

	return LaunchDescription([
		DeclareLaunchArgument(
			'serial_port', default_value='/dev/ldlidar', description='Serial port for the lidar'
		),
		DeclareLaunchArgument(
			'lidar_model', default_value='LDLiDAR_LD19', description='Lidar model to use'
		),
		ldlidar_include,
		*nodes,
	])

