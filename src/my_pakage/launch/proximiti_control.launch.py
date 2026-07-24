from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
		"""Launch mapping, control and location nodes for a static test.

		Nodes started (package='my_pakage', executable=...):
			- simulated_sensor
			- proximiti_direccion
			- direccion
			- lidar_suscriber
			- lidar_image_creator
			- lidar_processing
			- bycicle_mode
			- imu_suscriber
			- ldlidar_listener
		"""

		return LaunchDescription([
				# Control (proximity -> PID -> servo)
				Node(package='my_pakage', executable='simulated_sensor', name='simulated_sensor', output='screen'),
				Node(package='my_pakage', executable='proximiti_direccion', name='proximiti_direccion', output='screen'),
				Node(package='my_pakage', executable='direccion', name='direccion', output='screen'),

				# Mapping (LiDAR processing)
				Node(package='my_pakage', executable='lidar_suscriber', name='lidar_suscriber', output='screen'),
				Node(package='my_pakage', executable='lidar_image_creator', name='lidar_image_creator', output='screen'),
				Node(package='my_pakage', executable='lidar_processing', name='lidar_processing', output='screen'),

				# Location / state estimation
				Node(package='my_pakage', executable='bycicle_mode', name='bycicle_mode', output='screen'),
				Node(package='my_pakage', executable='imu_suscriber', name='imu_suscriber', output='screen'),

				# LiDAR listener (driver bringup if available)
				Node(package='my_pakage', executable='ldlidar_listener', name='ldlidar_listener', output='screen'),
		])
