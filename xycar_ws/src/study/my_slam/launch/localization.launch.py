"""만들어둔 지도 위에서 측위.

    ros2 launch my_slam localization.launch.py map:=/home/user/track_map

map 인자는 **확장자 없는 경로**다 (track_map.pgm / track_map.yaml 을 가리킴).

출발위치:
    매핑을 출발선에서 시작했다면 지도 원점 (0,0,0) 이 곧 출발 위치다.
    규정상 출발 위치가 고정이므로 config/localization.yaml 의 map_start_pose
    를 그대로 쓰면 된다. 차를 출발선에 정확히 놓고 시작할 것.
    다른 위치에서 시작해야 하면 start_x/start_y/start_yaw 인자로 덮어쓴다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('my_slam')
    rf2o_params = os.path.join(share, 'config', 'rf2o.yaml')
    slam_params = os.path.join(share, 'config', 'localization.yaml')
    default_map = os.path.join(share, 'maps', 'track_map')

    args = [
        DeclareLaunchArgument('map', default_value=default_map,
                              description='지도 경로 (확장자 제외)'),
        # [실측] mapping.launch.py 와 동일해야 한다 (다르면 지도와 측위가 어긋난다).
        #   축거 33.3cm + 앞바퀴축에서 라이다까지 8.5cm = 0.418 m
        DeclareLaunchArgument('laser_x', default_value='0.418',
                              description='base_link(뒷바퀴축)->laser x (m)'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.10',
                              description='지면->라이다 중심 높이 (m). 실측 10cm'),
        DeclareLaunchArgument('laser_frame', default_value='laser_frame'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('laser_frame'),
        ],
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params],
    )

    slam = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'map_file_name': LaunchConfiguration('map')},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [static_tf_laser, rf2o, slam, rviz])
