import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('xycar_motor'), 'config', 'vesc.yaml')

    return LaunchDescription([
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='xycar_motor',
            executable='xycar_motor',
            name='xycar_motor',
            parameters=[config],
            output='screen',
        ),
    ])
