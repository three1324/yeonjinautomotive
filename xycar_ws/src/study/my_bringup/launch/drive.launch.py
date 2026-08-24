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

    debug            OpenCV 파이프라인 뷰어(카메라 시점)를 띄울지 (기본 **false**)
    rviz             RViz2(공간 시점)까지 같이 띄울지 (기본 **false**)

★ [2026-08-24] debug/rviz 기본값을 **false 로 되돌렸다.**
  시각화는 CPU 를 쓰는데 이 노드들은 이미 YOLO 추론이 병목이다. 게다가
  debug/rviz 를 켜면 perception_node 가 /debug_image 까지 그리므로,
  추론 위에 그리기 비용이 더해져 제어 주기가 흔들린다.
  실전·기록 주행이 기본이므로 꺼진 쪽이 기본값이어야 한다.

        ros2 launch my_bringup drive.launch.py                # 시각화 없음(기본)
        ros2 launch my_bringup drive.launch.py debug:=true    # OpenCV 뷰어만
        ros2 launch my_bringup drive.launch.py rviz:=true     # RViz2 까지

  뜨는 것(기본): 카메라 + 라이다 드라이버, perception_node,
                 rubbercone_node, driver_node.

안전: **이 launch 만으로 주행 준비가 끝난다** (require_enable=false, 2026-08-22).
      /drive_enable 을 따로 쏠 필요가 없다. 안전장치가 사라진 게 아니라
      신호등으로 옮겨간 것이다 — fsm.auto_start 가 false 라 WAIT_LIGHT 에서
      신호 확정까지 서 있는다.
      ★ [2026-08-22] 출발 신호가 **좌회전 화살표(LEFT)** 다
        (fsm.start_on_green=false / start_on_left=true). 초록불로는 안 나간다.
      ⚠️ 뒤집어 말하면 **신호등 오검출 한 번에 출발한다.** 벤치에서 차를
         들어올리고 테스트할 때는 drive_params.yaml 의 require_enable 을
         true 로 되돌리고, 그때는 다음이 다시 필요하다:
             ros2 topic pub --once /drive_enable std_msgs/msg/Bool '{data: true}'

⚠️ 모터는 이 launch 에 없다. 이 차량의 모터/VESC 는 ROS1 도커
   (ros1_container + ros1_bridge)가 담당하며 **따로 띄워야** 한다
   (xycar_ws/etc/motor_vesc/motor 참고). 그게 안 떠 있으면 노드는 다 뜨고
   /xycar_motor 로 명령도 나가지만 바퀴는 안 움직인다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, OrSubstitution,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _pick_model(perception_share):
    """TensorRT 엔진이 있으면 그것을, 없으면 PyTorch 가중치를 쓴다.

    엔진(.engine)은 **이 젯슨에서 만든 것만 유효하다** — TensorRT/JetPack 버전과
    GPU 아키텍처에 종속이라 다른 보드로 옮기면 로드에 실패한다. 그래서 git 에
    올리지 않고(.gitignore), 보드마다 한 번씩 만든다:

        cd my_perception && python3 -c "from ultralytics import YOLO; \
            YOLO('models/best5.pt').export(format='engine', half=True, imgsz=640, device=0)"

    엔진이 없으면 자동으로 .pt 로 떨어지므로, 변환을 안 한 보드에서도 그냥 돈다.
    """
    engine = os.path.join(perception_share, 'models', 'best5.engine')
    if os.path.exists(engine):
        return engine
    return os.path.join(perception_share, 'models', 'best5.pt')


def generate_launch_description():
    bringup_share = get_package_share_directory('my_bringup')
    perception_share = get_package_share_directory('my_perception')

    default_params = os.path.join(bringup_share, 'config', 'drive_params.yaml')
    default_model = _pick_model(perception_share)

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
        # [2026-08-24] 기본값 true -> **false**. 실전은 꺼져 있어야 한다.
        #   켜려면: ros2 launch my_bringup drive.launch.py debug:=true
        # ⚠️ 켜면 perception_node 가 /debug_image 까지 그리므로 YOLO 추론
        #    위에 그리기 비용이 더해져 제어 주기가 흔들린다.
        DeclareLaunchArgument('debug', default_value='false'),
        # RViz2(공간 시점: 라이다 포인트클라우드/오도메트리/경로)까지 같이 띄운다.
        # 켜면 debug 와 무관하게 /debug_image 도 자동으로 켜진다(위 show_debug_image).
        # [2026-08-24] 기본값 true -> **false**.
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
    #
    # ★ 반드시 GroupAction(scoped=True) 로 감싼다 (2026-08-21).
    #   IncludeLaunchDescription 의 launch_arguments 는 **현재 스코프에**
    #   SetLaunchConfiguration 을 깔아버린다. 감싸지 않으면 여기서 못박은
    #   ydlidar.yaml 이 params_file 을 통째로 덮어써서, 아래에 오는 우리 노드
    #   4개(perception/obstacle/rubbercone/driver)까지 전부 ydlidar.yaml 을
    #   받는다. 그러면 drive_params.yaml 의 값이 하나도 안 들어가고 전부
    #   코드 기본값으로 뜬다 — 예컨대 rubbercone_node 의 drive_topic 이
    #   기본값 'xycar_motor' 로 남아 driver_node 와 둘이 모터 토픽에 쏘고,
    #   콘 구간이 아닌데도 차가 라이다 명령대로 움직인다. 실제로 그렇게 됐다.
    lidar = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('xycar_lidar'), 'launch', 'xycar_lidar.launch.py'])),
            launch_arguments={
                'params_file': PathJoinSubstitution(
                    [FindPackageShare('xycar_lidar'), 'params', 'ydlidar.yaml']),
            }.items(),
        ),
    ], scoped=True, forwarding=True, condition=IfCondition(use_sensors))

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
    # ※ obstacle_node 는 없다 (2026-08-21 삭제). 라이다 복도 추정/섹터 거리는
    #   주행에 쓰지 않기로 했고, 시각화만을 위해 라이다를 상시 돌릴 이유가 없다.
    #   라이다를 쓰는 노드는 아래 rubbercone_node 하나뿐이다.
    # --- 라바콘 구간 전담 (2026-08-22 개편판: 사슬 중심선 + 뒤축 Pure Pursuit) ---
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
                                     perception, rubbercone, driver, viz])
