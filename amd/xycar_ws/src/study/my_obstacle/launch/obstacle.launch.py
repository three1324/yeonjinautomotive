"""obstacle_node 단독 실행 (디버깅용).

    ros2 launch my_obstacle obstacle.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('my_bringup'), 'config', 'drive_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='my_obstacle',
            executable='obstacle_node',
            name='obstacle_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
