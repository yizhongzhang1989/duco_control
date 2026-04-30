from setuptools import find_packages, setup

package_name = 'duco_cartesian_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/cartesian_control.launch.py',
            'launch/cartesian_control_real.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/fzi_zero_gravity.yaml',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='FZI cartesian_force_controller setup + engage/disengage UX '
                'and safety supervisor for the Duco GCR5_910 arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartesian_control_node = duco_cartesian_control.control_node:main',
        ],
    },
)
