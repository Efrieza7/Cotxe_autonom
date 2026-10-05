from setuptools import find_packages, setup

package_name = 'my_pakage'

# Ensure the bundled `fsd_path_planning` library (vendored under
# nodes/path_planning/ft-fsd-path-planning-main/fsd_path_planning) is
# installed as a top-level package so imports like
# `from fsd_path_planning import ...` work at runtime.
extra_packages = ['fsd_path_planning']
setup(
    name=package_name,
    version='2.0.1',
    # include vendored fsd_path_planning subpackages so their modules
    # (full_pipeline, calculate_path, utils, etc.) are installed
    packages=(
        find_packages(exclude=['test'])
        + find_packages(where='nodes/path_planning/ft-fsd-path-planning-main')
    ),
    package_dir={
        # map the top-level package name to the vendored location
        'fsd_path_planning': 'nodes/path_planning/ft-fsd-path-planning-main/fsd_path_planning'
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/ldlidar_integration.launch.py',
                'launch/proximity_control.launch.py',
                'launch/path_planner_bridge.launch.py',
                'launch/full_car.launch.py',
                'launch/simulation_mapping.launch.py',
                'launch/simulation_full.launch.py',
            ],
        ),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'scikit-learn',
        'numba<0.60',
        'typing-extensions',
    ],
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
            'my_node = nodes.templates.my_node:main',
            'my_publisher = nodes.templates.publisher:main',
            'my_subscriber = nodes.templates.subscriber:main',
            'proximity_steering = nodes.control.proximity_sensors.proximity_steering:main',
            'proximity_reader = nodes.control.proximity_sensors.proximity_reader:main',
            'steering = nodes.control.steering:main',
            'imu_reader = nodes.location.imu.imu_reader:main',
            'lidar_subscriber = nodes.mapping.lidar.lidar_subscriber:main',
            'lidar_image_creator = nodes.mapping.lidar.lidar_image_creator:main',
            'ldlidar_listener = nodes.ldlidar_listener:main',
            'lidar_processing = nodes.mapping.lidar.lidar_processing:main',
            'bicycle_model = nodes.location.bicycle_model.bicycle_model:main',
            'cone_map_viz = nodes.mapping.lidar.cone_map_viz:main',
            'motor_reader = nodes.location.motor_reader:main',
            'path_planning = nodes.path_planning.path_planner_bridge_node:main',
            'path_planner_bridge = nodes.path_planning.path_planner_bridge_node:main',
            'path_follower = nodes.control.path_follower:main',
            'speed_control = nodes.control.speed_control:main',
            'start_car = launcher.start_car:main',
            'test_servo = launcher.test_servo:main',

        ],
    },
)
