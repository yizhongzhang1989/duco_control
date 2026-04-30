from setuptools import find_packages, setup

package_name = 'common'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Centralized configuration loader and workspace utilities for the duco_control project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
