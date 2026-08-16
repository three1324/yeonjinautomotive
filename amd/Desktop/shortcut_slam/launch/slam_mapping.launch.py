"""
숏컷 구간 SLAM 매핑 검증용 launch 파일.

흐름: 라이다(/scan) -> rf2o(스캔 매칭 오도메트리) -> slam_toolbox(매핑) -> RViz
- 주행 코드(race_manager, left_turn 등)와는 완전히 독립적으로 동작한다.
- scan_check_node가 /scan을 직접 구독해서 하드웨어 데이터 정상 여부를 로그로 보여준다.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('shortcut_slam')
    rf2o_params = os.path.join(pkg_share, 'config', 'rf2o_params.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_view.rviz')

    # TODO 뒷바퀴 축으로부터 라이다까지의 거리
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.43', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
        ],
    )

    scan_check = Node(
        package='shortcut_slam',
        executable='scan_check_node',
        name='scan_check_node',
        output='screen',
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        static_tf_laser,
        scan_check,
        rf2o_node,
        slam_toolbox_node,
        rviz_node,
    ])

