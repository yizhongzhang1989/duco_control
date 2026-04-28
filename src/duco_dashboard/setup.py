import os

from setuptools import find_packages, setup

package_name = 'duco_dashboard'


def package_files(directory):
    paths = []
    for root, _, files in os.walk(directory):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, root)
        paths.append((install_dir, [os.path.join(root, name) for name in files]))
    return paths

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dashboard.launch.py']),
    ] + package_files('static'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Web dashboard for monitoring Duco robot state.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = duco_dashboard.dashboard_node:main',
        ],
    },
)