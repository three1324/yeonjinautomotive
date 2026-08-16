"""전체 주행 통합 launch.

    ros2 launch my_bringup drive.launch.py

인자:
    params_file   파라미터 yaml 경로 (기본: my_bringup/config/drive_params.yaml)
    sensors       센서 드라이버(카메라/라이다)도 같이 띄울지 (기본 true)
    motor         모터 스택(vesc + xycar_motor)도 같이 띄울지 (기본 true)
    slam          SLAM 측위를 같이 띄울지 (기본 false — 1단계에서는 불필요)

안전: driver_node 는 require_enable=true 이므로 이 launch 를 띄워도 차는 움직이지 않는다.
      출발시키려면 별도로:
          ros2 topic pub --once /drive_enable std_msgs/Bool '{data: true}'
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
    default_model = os.path.join(perception_share, 'models', 'best5.pt')

    params_file = LaunchConfiguration('params_file')
    use_sensors = LaunchConfiguration('sensors')
    use_motor = LaunchConfiguration('motor')
    use_slam = LaunchConfiguration('slam')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('motor', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
    ]

    # --- 센서 드라이버 ---
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

    # --- 모터 스택 (vesc_driver + ackermann_to_vesc + vesc_to_odom + xycar_motor) ---
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
        parameters=[params_file, {'model_path': default_model}],
    )
    obstacle = Node(
        package='my_obstacle',
        executable='obstacle_node',
        name='obstacle_node',
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

    return LaunchDescription(args + [cam, lidar, motor, slam,
                                     perception, obstacle, driver])
