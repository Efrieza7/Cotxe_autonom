from setuptools import find_packages, setup

package_name = 'my_pakage'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ldlidar_integration.launch.py']),
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
            'my_node = my_pakage.templates_node.my_node:main',
            'my_publisher = my_pakage.templates_node.publisher:main',
            'my_suscriber = my_pakage.templates_node.suscriber:main',
            'proximiti_direccion = my_pakage.control.proximiti_sesors.proximiti_direccion:main',
            'proximiti_reader = my_pakage.control.proximiti_sesors.proximiti_reader:main',
            'direccion = my_pakage.control.direccion:main',
            'imu_suscriber = my_pakage.location.imu.imu_suscriber:main',
            'lidar_suscriber = my_pakage.maping.lidar.lidar_suscriber:main',
            'lidar_image_creator = my_pakage.maping.lidar.lidar_image_creator:main',
            'ldlidar_listener = my_pakage.ldlidar_listener:main',
            'lidar_processing = my_pakage.maping.lidar.lidar_processing:main',
            'bycicle_mode = my_pakage.location.Bycicle_mode.bycicle_mode:main',
            'cons_map_viz = my_pakage.maping.lidar.cons_map_viz:main',

        ],
    },
)
