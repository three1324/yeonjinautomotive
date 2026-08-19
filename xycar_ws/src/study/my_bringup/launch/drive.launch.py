"""전체 주행 통합 launch.

    ros2 launch my_bringup drive.launch.py

인자:
    params_file   파라미터 yaml 경로 (기본: my_bringup/config/drive_params.yaml)
    sensors       센서 드라이버(카메라/라이다)도 같이 띄울지 (기본 true)
    motor         ROS2 판 모터 스택(vesc + xycar_motor)도 같이 띄울지 (기본 false)
                  ⚠️ 이 차량의 모터는 ROS1 도커(ros1_container + ros1_bridge)가 담당한다.
                  ROS2 판은 쓰지 않는다 — 동시에 띄우면 /dev/ttyMOTOR 를 두 프로세스가
                  잡으려 해서 충돌한다. 근거: JETSON_ROS1_DOCKER_MOTOR.md §8-(3)
    slam          SLAM 측위를 같이 띄울지 (기본 false — 1단계에서는 불필요)

    debug            OpenCV 파이프라인 뷰어(카메라 시점)를 띄울지 (기본 false)
    rviz             RViz2(공간 시점)까지 같이 띄울지 (기본 false)
                     rviz:=true 면 debug 값과 무관하게 /debug_image 도 자동으로 켜지고
                     OpenCV 뷰어와 RViz2 두 창이 함께 뜬다. 즉 명령 한 줄이면 된다:
                         ros2 launch my_bringup drive.launch.py rviz:=true
                     ⚠️ 시각화는 CPU 를 쓴다. 기록 주행/실전에서는 둘 다 false 로 둘 것.

안전: driver_node 는 require_enable=true 이므로 이 launch 를 띄워도 차는 움직이지 않는다.
      출발시키려면 별도로:
          ros2 topic pub --once /drive_enable std_msgs/msg/Bool '{data: true}'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, OrSubstitution,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('my_bringup')
    perception_share = get_package_share_directory('my_perception')

    default_params = os.path.join(bringup_share, 'config', 'drive_params.yaml')
    default_model = os.path.join(perception_share, 'models', 'best5.pt')

    params_file = LaunchConfiguration('params_file')
    use_sensors = LaunchConfiguration('sensors')
    use_motor = LaunchConfiguration('motor')
    use_slam = LaunchConfiguration('slam')
    use_debug = LaunchConfiguration('debug')
    use_rviz = LaunchConfiguration('rviz')
    # rviz 를 켜면 RViz Image 패널에 띄울 /debug_image 도 있어야 한다.
    show_debug_image = OrSubstitution(use_debug, use_rviz)

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('motor', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false'),
        # true 로 켜면 perception_node 가 /debug_image 를 내고 my_debug 뷰어 창이 뜬다.
        DeclareLaunchArgument('debug', default_value='false'),
        # RViz2(공간 시점: 라이다 포인트클라우드/오도메트리/경로)까지 같이 띄운다.
        # 켜면 debug 와 무관하게 /debug_image 도 자동으로 켜진다(위 show_debug_image).
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    # --- 센서 드라이버 ---
    cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('xycar_cam'), 'launch', 'xycar_cam.launch.py'])),
        condition=IfCondition(use_sensors),
    )
    # ⚠️ xycar_lidar.launch.py 도 'params_file' 이라는 같은 이름의 인자를 선언한다.
    #    이미 위에서 우리가 params_file 을 선언해 두었으므로, include 쪽
    #    DeclareLaunchArgument 의 기본값(ydlidar.yaml)은 무시되고 drive_params.yaml
    #    이 그대로 넘어간다. 거기엔 xycar_lidar_node 항목이 없어서 노드가
    #    컴파일 기본값(/dev/ydlidar, 230400)으로 뜨고 "cannot bind to the specified
    #    serial port" 로 죽는다. 그래서 여기서 라이다 yaml 을 명시적으로 못박는다.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('xycar_lidar'), 'launch', 'xycar_lidar.launch.py'])),
        launch_arguments={
            'params_file': PathJoinSubstitution(
                [FindPackageShare('xycar_lidar'), 'params', 'ydlidar.yaml']),
        }.items(),
        condition=IfCondition(use_sensors),
    )

    # --- ROS2 판 모터 스택 (기본 꺼짐) ---
    # 이 차량의 모터는 ROS1 도커가 잡는다(§8-(3)). 여기를 켜면 포트가 충돌한다.
    motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('xycar_motor'), 'launch', 'xycar_motor.launch.py'])),
        condition=IfCondition(use_motor),
    )

    # --- SLAM 측위 (2단계에서 사용) ---
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('my_slam'), 'launch', 'localization.launch.py'])),
        condition=IfCondition(use_slam),
    )

    # --- 우리 노드 3개 ---
    perception = Node(
        package='my_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[params_file, {
            'model_path': default_model,
            'publish_debug_image': ParameterValue(show_debug_image, value_type=bool),
        }],
    )
    obstacle = Node(
        package='my_obstacle',
        executable='obstacle_node',
        name='obstacle_node',
        output='screen',
        parameters=[params_file],
    )
    # --- 라바콘 구간 전담 (팀원 실차 검증 구현을 그대로 가져온 노드) ---
    # 스스로 구간을 판정하고 조향·속도까지 만든다. driver_node 는 구간일 때
    # 그 명령(/cone_cmd)을 그대로 통과시키기만 한다.
    # drive_topic 을 xycar_motor 가 아니라 cone_cmd 로 돌리는 것이 핵심 —
    # 모터 토픽에 발행자가 둘이 되면 서로 다른 명령이 섞여 차가 요동친다.
    rubbercone = Node(
        package='my_obstacle',
        executable='rubbercone_node',
        name='rubbercone_node',
        output='screen',
        parameters=[params_file],
    )
    driver = Node(
        package='my_driver',
        executable='driver_node',
        name='driver_node',
        output='screen',
        parameters=[params_file],
    )
    # --- 시각화 (OpenCV 카메라 뷰어 + RViz2) ---
    # debug 든 rviz 든 켜지면 perception_node 가 /debug_image 를 내야 하므로
    # 위 publish_debug_image 에 두 인자의 OR 을 넘겼다. 여기서는 창만 띄운다.
    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('my_debug'), 'launch', 'viz.launch.py'])),
        launch_arguments={
            'rviz': use_rviz,
            'pipeline_view': 'true',
            'params_file': params_file,
        }.items(),
        condition=IfCondition(show_debug_image),
    )

    return LaunchDescription(args + [cam, lidar, motor, slam,
                                     perception, obstacle, rubbercone, driver, viz])
