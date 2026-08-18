from setuptools import find_packages, setup

package_name = 'codi_principal'

setup(
    name=package_name,
    version='2.0.1',
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
            'my_node = codi_principal.templates_node.my_node:main',
            'my_publisher = codi_principal.templates_node.publisher:main',
            'my_suscriber = codi_principal.templates_node.suscriber:main',
            'proximiti_direccion = codi_principal.control.proximiti_sesors.proximiti_direccion:main',
            'proximiti_reader = codi_principal.control.proximiti_sesors.proximiti_reader:main',
            'direccion = codi_principal.control.direccion:main',
            'imu_suscriber = codi_principal.location.imu.imu_suscriber:main',
            'lidar_suscriber = codi_principal.maping.lidar.lidar_suscriber:main',
            'lidar_image_creator = codi_principal.maping.lidar.lidar_image_creator:main',
            'ldlidar_listener = codi_principal.ldlidar_listener:main',
            'lidar_processing = codi_principal.maping.lidar.lidar_processing:main',
            'bycicle_mode = codi_principal.location.Bycicle_mode.bycicle_mode:main',
            'cons_map_viz = codi_principal.maping.lidar.cons_map_viz:main',
            'motor_reader = codi_principal.control.motor.motor_reader:main',
            'path_planning = codi_principal.path_planning.ros_node:main',

        ],
    },
)
