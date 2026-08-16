"""SLAM 지도 생성.

    ros2 launch my_slam mapping.launch.py

절차:
    1. 차를 **출발선에 정확히** 놓는다 (지도 원점 = 출발 위치가 된다)
    2. 이 launch 실행
    3. 트랙을 한 바퀴 이상 천천히 주행 (수동 조종 권장)
    4. 지도 저장:
           ros2 run nav2_map_server map_saver_cli -f ~/track_map
       생성된 track_map.pgm / track_map.yaml 을 my_slam/maps/ 로 옮긴다

인자:
    laser_x/y/z  base_link 기준 라이다 장착 위치 (m)  ★ 실측 필수
    use_ekf      rf2o 대신 EKF 융합 오도메트리 사용 (2단계)
    rviz         RViz 를 함께 띄울지
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('my_slam')
    rf2o_params = os.path.join(share, 'config', 'rf2o.yaml')
    slam_params = os.path.join(share, 'config', 'mapping.yaml')
    ekf_params = os.path.join(share, 'config', 'ekf.yaml')

    use_ekf = LaunchConfiguration('use_ekf')
    use_rviz = LaunchConfiguration('rviz')

    args = [
        # ★ 실측 필수 ★
        # 기존 shortcut_slam 에는 x=0.43 으로 되어 있었으나, 1/10 스케일 RC카의
        # wheelbase 가 26cm 안팎인 것을 감안하면 차체 밖이라 의심스럽다.
        # 뒷바퀴 축(base_link) 에서 라이다까지의 실제 거리를 재서 넣을 것.
        # 이 값이 틀리면 스캔이 어긋난 위치에 찍혀 지도가 흐려지고 측위 오차가 생긴다.
        DeclareLaunchArgument('laser_x', default_value='0.20',
                              description='base_link->laser x (m) ★실측 필요'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.10',
                              description='base_link->laser z (m) ★실측 필요'),
        DeclareLaunchArgument('laser_frame', default_value='laser_frame'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
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

    # EKF 를 쓸 때는 rf2o 가 TF 를 발행하면 충돌한다 (EKF 가 odom->base_link 를 낸다).
    # condition 은 생성자에만 넘길 수 있어서 두 벌을 만들고 하나만 실행되게 한다.
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params],
        condition=UnlessCondition(use_ekf),
    )
    rf2o_no_tf = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params, {'publish_tf': False}],
        condition=IfCondition(use_ekf),
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
        condition=IfCondition(use_ekf),
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [static_tf_laser, rf2o, rf2o_no_tf, ekf, slam, rviz])
