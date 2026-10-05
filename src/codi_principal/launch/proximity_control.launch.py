from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
		"""Launch mapping, control and location nodes for a static test.

		Nodes started (package='my_pakage', executable=...):
			- proximity_reader
			- proximity_steering
			- steering
			- lidar_subscriber
			- lidar_image_creator
			- lidar_processing
			- bicycle_model
			- imu_reader
			- ldlidar_listener
		"""

		return LaunchDescription([
				# Control (proximity -> PID -> servo)
				Node(package='my_pakage', executable='proximity_reader', name='proximity_reader', output='screen'),
				Node(package='my_pakage', executable='proximity_steering', name='proximity_steering', output='screen'),
				Node(package='my_pakage', executable='steering', name='steering', output='screen'),

				# Mapping (LiDAR processing)
				Node(package='my_pakage', executable='lidar_subscriber', name='lidar_subscriber', output='screen'),
				Node(package='my_pakage', executable='lidar_image_creator', name='lidar_image_creator', output='screen'),
				Node(package='my_pakage', executable='lidar_processing', name='lidar_processing', output='screen'),

				# Location / state estimation
				Node(package='my_pakage', executable='bicycle_model', name='bicycle_model', output='screen'),
				Node(package='my_pakage', executable='imu_reader', name='imu_reader', output='screen'),

				# LiDAR listener (driver bringup if available)
				Node(package='my_pakage', executable='ldlidar_listener', name='ldlidar_listener', output='screen'),
		])
