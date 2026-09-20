from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Mapping-only nodes needed on top of `car_simulator`, without any hardware or
    `bycicle_mode` (the simulator already publishes ground-truth `/pose` and
    `/bicycle_mode/pose`, so running `bycicle_mode` here would fight it for those topics).

    Nodes started (package='my_pakage', executable=...):
        - lidar_image_creator  (turns /ldlidar_node/scan into global /ldlidar_node/scan_xy)
        - lidar_processing     (clusters scan_xy into /ldlidar_node/cons_map)
        - cons_map_viz         (RViz MarkerArray of the detected cons_map)
    """

    return LaunchDescription([
        Node(package='my_pakage', executable='lidar_image_creator', name='lidar_image_creator', output='screen'),
        Node(package='my_pakage', executable='lidar_processing', name='lidar_processing', output='screen'),
        Node(package='my_pakage', executable='cons_map_viz', name='cons_map_viz', output='screen'),
    ])
