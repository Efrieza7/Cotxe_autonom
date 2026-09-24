from setuptools import find_packages, setup

package_name = 'car_simulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/car_simulator.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/car.urdf']),
        ('share/' + package_name + '/rviz', ['rviz/car_simulator.rviz']),
        ('share/' + package_name + '/maps', ['maps/sample_track.yaml', 'maps/complex_track.yaml', 'maps/complex_track_clean.yaml', 'maps/circuit_realista.yaml', 'maps/circuit_realista_V2.yaml', 'maps/circuit_BCN.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Efrieza',
    maintainer_email='senricbe@gmail.com',
    description='RViz simulator: car model, ground-truth cone map and simulated LiDAR '
                 'detections limited to a configurable range.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'car_simulator_node = car_simulator.car_simulator_node:main',
            'lidar_simulator_node = car_simulator.lidar_simulator_node:main',
            'cone_map_publisher_node = car_simulator.cone_map_publisher_node:main',
            'path_visualizer_node = car_simulator.path_visualizer_node:main',
            'generate_map = car_simulator.generate_map:main',
        ],
    },
)
