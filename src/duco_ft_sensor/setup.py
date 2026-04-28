from setuptools import find_packages, setup

package_name = 'duco_ft_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ft_sensor.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial>=3.5'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='ROS 2 driver for the Duco 6-DoF force/torque sensor.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ft_sensor_node = duco_ft_sensor.ft_sensor_node:main',
            'read_ft_sensor = duco_ft_sensor.cli:main',
        ],
    },
)
