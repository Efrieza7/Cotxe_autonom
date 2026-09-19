from setuptools import find_packages, setup

package_name = 'my_pakage'

setup(
    name=package_name,
    version='2.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/ldlidar_integration.launch.py',
                'launch/proximiti_control.launch.py',
                'launch/path_planner_bridge.launch.py',
                'launch/complet_code.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Efrieza',
    maintainer_email='sernicbe@gmail.com',
    description='Projecte TDR',
    license='Mudle Catala',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = nodes.templates_node.my_node:main',
            'my_publisher = nodes.templates_node.publisher:main',
            'my_suscriber = nodes.templates_node.suscriber:main',
            'proximiti_direccion = nodes.control.proximiti_sesors.proximiti_direccion:main',
            'proximiti_reader = nodes.control.proximiti_sesors.proximiti_reader:main',
            'direccion = nodes.control.direccion:main',
            'imu_suscriber = nodes.location.imu.imu_suscriber:main',
            'lidar_suscriber = nodes.maping.lidar.lidar_suscriber:main',
            'lidar_image_creator = nodes.maping.lidar.lidar_image_creator:main',
            'ldlidar_listener = nodes.ldlidar_listener:main',
            'lidar_processing = nodes.maping.lidar.lidar_processing:main',
            'bycicle_mode = nodes.location.Bycicle_mode.bycicle_mode:main',
            'cons_map_viz = nodes.maping.lidar.cons_map_viz:main',
            'motor_reader = nodes.location.motor_reader:main',
            'path_planning = nodes.path_planning.path_planner_bridge_node:main',
            'path_planner_bridge = nodes.path_planning.path_planner_bridge_node:main',
            'path_follower = nodes.control.path_follower:main',
            'speed_control = nodes.control.speed_control:main',
            'launch_complet = launcher.launch_complet:main',

        ],
    },
)
