
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lab1_controller')
    params_file = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        Node(
            package='lab1_controller',
            executable='go_to_pose_server',
            name='go_to_pose_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='lab1_controller',
            executable='reset_server',
            name='reset_server',
            output='screen'
        ),

        Node(
            package='lab1_controller',
            executable='square_server',
            name='square_server',
            output='screen'
        ),
    ])
