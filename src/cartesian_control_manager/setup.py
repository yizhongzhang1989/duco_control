from setuptools import find_packages, setup

package_name = 'cartesian_control_manager'

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
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Robot-agnostic orchestrator for FZI cartesian controllers: '
                'engage/disengage UX, wrench relay, and safety supervisor.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartesian_control_node = cartesian_control_manager.control_node:main',
        ],
    },
)
