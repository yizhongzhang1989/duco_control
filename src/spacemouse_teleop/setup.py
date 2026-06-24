from glob import glob
import os

from setuptools import setup

package_name = 'spacemouse_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yizhong Zhang',
    maintainer_email='yizhong@todo.todo',
    description='SpaceMouse Cartesian jog bridge (twist -> PoseStamped target).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = spacemouse_teleop.servo_node:main',
            'dashboard_node = spacemouse_teleop.dashboard_node:main',
        ],
    },
)
