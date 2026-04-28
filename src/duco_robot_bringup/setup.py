from glob import glob
from setuptools import find_packages, setup

package_name = 'duco_robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Project-owned launch wrappers for Duco robot bringup.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)