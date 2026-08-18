"""주행 시각화 전용 launch — RViz2 + viz_node (+ 카메라 뷰어).

단독 실행 (주행 스택은 이미 떠 있을 때):

    ros2 launch my_debug viz.launch.py

주행 스택과 한 번에 (권장):

    ros2 launch my_bringup drive_amd.launch.py rviz:=true

뜨는 창은 두 개다. 보는 축이 달라서 둘 다 필요하다.
    1) "xycar pipeline" (OpenCV)  — 카메라 시점. 좌: YOLO 검출 / 우: 차선 추정,
                                     하단 바에 판단(FSM)·계획·제어 텍스트.
    2) RViz2                       — 공간 시점(top-down). 라이다 포인트클라우드,
                                     오도메트리, 지나온 궤적/예측 경로/기준 경로.

인자:
    rviz            RViz2 를 띄울지            (기본 true)
    pipeline_view   OpenCV 뷰어를 띄울지        (기본 true)
    rviz_config     RViz 설정 파일 경로         (기본 my_debug/config/drive.rviz)
    params_file     viz_node 파라미터 yaml      (기본 my_bringup/config/drive_params.yaml)
    laser_tf        base_link->laser_frame 정적 TF 를 여기서 낼지 (기본 true)
                    RViz 는 Fixed Frame(odom) 으로 스캔을 옮겨 그려야 하므로 이 TF 가
                    없으면 라이다가 화면에 안 나온다. my_slam 을 같이 띄웠다면 그쪽이
                    같은 TF 를 이미 내므로 false 로 꺼도 된다(값이 같아 켜둬도 무해).

⚠️ 1) 창은 perception_node 가 `publish_debug_image:=true` 여야 내용이 찬다.
   통합 launch 의 `rviz:=true` / `debug:=true` 는 그 값을 자동으로 켜준다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug_share = get_package_share_directory('my_debug')
    bringup_share = get_package_share_directory('my_bringup')

    default_rviz = os.path.join(debug_share, 'config', 'drive.rviz')
    default_params = os.path.join(bringup_share, 'config', 'drive_params.yaml')

    use_rviz = LaunchConfiguration('rviz')
    use_view = LaunchConfiguration('pipeline_view')
    use_laser_tf = LaunchConfiguration('laser_tf')

    args = [
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('pipeline_view', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('laser_tf', default_value='true'),
        # [실측] my_slam/launch/localization.launch.py 와 같은 값이어야 한다.
        #   축거 33.3cm + 앞바퀴축에서 라이다까지 8.5cm = 0.418 m, 지면->라이다 10cm
        DeclareLaunchArgument('laser_x', default_value='0.418'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.10'),
        DeclareLaunchArgument('laser_frame', default_value='laser_frame'),
    ]

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='viz_base_to_laser_tf',
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('laser_frame'),
        ],
        condition=IfCondition(use_laser_tf),
    )

    viz = Node(
        package='my_debug',
        executable='viz_node',
        name='viz_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    pipeline_view = Node(
        package='my_debug',
        executable='pipeline_view_node',
        name='pipeline_view_node',
        output='screen',
        condition=IfCondition(use_view),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [static_tf_laser, viz, pipeline_view, rviz])
