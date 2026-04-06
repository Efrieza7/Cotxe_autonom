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
            'my_node = my_pakage.my_node:main',
            'my_publisher = my_pakage.publisher:main',
            'my_suscriber = my_pakage.suscriber:main',
            'simulated_sensor = my_pakage.simulated_sensors:main',
            'proximiti_direccion = my_pakage.proximiti_sensors.proximiti_direccion:main'

        ],
    },
)
