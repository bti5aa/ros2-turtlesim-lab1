
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab1_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bti5a',
    maintainer_email='bti5a@example.com',
    description='ROS 2 lab controller for turtlesim actions and services',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_pose_server = lab1_controller.go_to_pose_server:main',
            'reset_server = lab1_controller.reset_server:main',
            'square_server = lab1_controller.square_server:main',
        ],
    },
)
