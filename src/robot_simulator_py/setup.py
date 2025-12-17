from setuptools import setup
import os
from glob import glob

package_name = 'robot_simulator_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Python controller node for robot simulation',
    license='Apache 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'controller_node = robot_simulator_py.controller_node:main',
            'mission_control = robot_simulator_py.mission_control:main',
        ],
    },
)