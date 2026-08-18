"""AMD 보드 xycar(서울대 대회 차량)용 전체 주행 통합 launch.

    ros2 launch my_bringup drive_amd.launch.py

젯슨용 `drive.launch.py` 와 **노드 구성·코드가 완전히 동일**하고, 딱 두 가지만 다르다.

1. 모터 스택을 띄우지 않는다.
   젯슨에서는 우리가 `xycar_motor`(+vesc) 패키지를 직접 띄워 VESC 를 잡지만,
   AMD 차량은 모터 드라이버가 **별도 ROS1 도커** 안에서 이미 상시 동작 중이고
   `xycar_motor` 토픽을 구독한다. 그래서 study 쪽에서는 발행만 하면 되고,
   `xycar_motor` 패키지는 이 차량에 존재하지도 않는다
   (`drive.launch.py` 를 그대로 쓰면 FindPackageShare 가 그 패키지를 못 찾아 실패한다).

2. 모터 명령 스케일이 다르므로 `config/amd_overrides.yaml` 을 덧씌운다.
   토픽 계약([angle, speed], angle ±50)은 양쪽이 같지만 speed 의 실효 배율이 다르다.
   자세한 근거는 그 파일 주석 참고.

센서(`xycar_cam` / `xycar_lidar`)는 패키지명·launch 파일명이 젯슨 쪽과 동일해서
그대로 include 한다.

인자:
    params_file      주 파라미터 yaml (기본: my_bringup/config/drive_params.yaml)
    overrides_file   덧씌울 yaml     (기본: my_bringup/config/amd_overrides.yaml)
    sensors          카메라/라이다 드라이버도 같이 띄울지 (기본 true)
    slam             SLAM 측위를 같이 띄울지 (기본 false)
                     ⚠️ AMD 차량 도커에는 slam_toolbox / rf2o_laser_odometry 가
                        없을 수 있다. 없으면 true 로 두는 순간 launch 가 죽는다.

안전: driver_node 는 require_enable=true 이므로 이 launch 를 띄워도 차는 움직이지 않는다.
      출발시키려면 별도로:
          ros2 topic pub --once /drive_enable std_msgs/Bool '{data: true}'

⚠️ 이 차량에는 서울대 대회용 `race_*` 패키지가 함께 설치돼 있을 수 있다.
   `race_manager` 도 `xycar_motor` 를 발행하므로 **동시에 띄우면 두 노드가 같은
   토픽에 서로 다른 명령을 쏴서 차가 요동친다.** 반드시 하나만 실행할 것.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('my_bringup')
    perception_share = get_package_share_directory('my_perception')

    default_params = os.path.join(bringup_share, 'config', 'drive_params.yaml')
    default_overrides = os.path.join(bringup_share, 'config', 'amd_overrides.yaml')
    default_model = os.path.join(perception_share, 'models', 'best5.pt')

    params_file = LaunchConfiguration('params_file')
    overrides_file = LaunchConfiguration('overrides_file')
    use_sensors = LaunchConfiguration('sensors')
    use_slam = LaunchConfiguration('slam')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('overrides_file', default_value=default_overrides),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
    ]

    # --- 센서 드라이버 (벤더 패키지, 젯슨과 이름 동일) ---
    cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('xycar_cam'), 'launch', 'xycar_cam.launch.py'])),
        condition=IfCondition(use_sensors),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('xycar_lidar'), 'launch', 'xycar_lidar.launch.py'])),
        condition=IfCondition(use_sensors),
    )

    # --- SLAM 측위 (기본 off. 의존 패키지가 이 차량에 있을 때만 켤 것) ---
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('my_slam'), 'launch', 'localization.launch.py'])),
        condition=IfCondition(use_slam),
    )

    # --- 우리 노드 3개 (젯슨과 완전히 같은 코드) ---
    # 파라미터 적용 순서: 뒤에 오는 것이 앞을 덮는다.
    #   drive_params.yaml (주 튜닝값)  ->  amd_overrides.yaml (차량별 보정)  ->  model_path
    # overrides 파일에 없는 노드 섹션은 그냥 무시되므로 세 노드에 모두 넘겨도 안전하다.
    common_params = [params_file, overrides_file]

    perception = Node(
        package='my_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=common_params + [{'model_path': default_model}],
    )
    obstacle = Node(
        package='my_obstacle',
        executable='obstacle_node',
        name='obstacle_node',
        output='screen',
        parameters=common_params,
    )
    driver = Node(
        package='my_driver',
        executable='driver_node',
        name='driver_node',
        output='screen',
        parameters=common_params,
    )

    return LaunchDescription(args + [cam, lidar, slam,
                                     perception, obstacle, driver])
