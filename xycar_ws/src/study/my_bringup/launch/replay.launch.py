"""영상 파일로 파이프라인 전체를 실시간 재생 테스트.

    ros2 launch my_bringup replay.launch.py video:=/home/e-on/테스트용.mp4

카메라·라이다 드라이버와 **모터 스택을 일절 띄우지 않는다.** 대신
my_debug/video_pub_node 가 영상 파일을 /image_raw 로 원래 fps 에 맞춰 흘려보내고,
그 뒤로는 실차와 **완전히 같은 노드·같은 파라미터**가 돈다:

    video_pub_node -> /image_raw -> perception_node -> /lane /light /objects
                                                    -> driver_node -> /xycar_motor
                                                    -> 시각화 2창

무엇을 확인할 수 있나:
    - 이 보드가 영상의 fps 를 실제로 따라가는가 (perception_node 로그의 실측 fps).
      못 따라가면 실차에서 driver_node 가 stale 로 멈춘다 — 인지가 느린 보드에서
      실제로 났던 증상이다(drive_params.yaml 의 stale_timeout_sec 주석 참고).
    - 차선 추정·YOLO 검출이 이 영상에서 어떻게 나오는가 (좌/우 2분할 창).
    - FSM/계획/제어가 어떤 값을 내는가 (하단 텍스트 바 + RViz).

라이다가 없으므로 /scan /obstacle /corridor 는 오지 않는다. driver_node 의
front_dist 기본값이 99.0m(=전방 개활)이라 장애물 로직은 그냥 놀고, 차선추종만
검증된다. 그래서 obstacle_node 는 아예 띄우지 않는다.

인자:
    video           재생할 영상 파일 경로 (필수)
    params_file     주 파라미터 yaml   (기본: my_bringup/config/drive_params.yaml)
    overrides_file  덧씌울 yaml        (기본: params_file 과 동일 = 덮어쓰기 없음.
                    보드가 느려 재생이 밀리면 stale_timeout_sec 을 늘릴 것)
    model           YOLO 가중치        (기본: my_perception/models/best5.pt)
    loop            끝나면 처음부터 다시 (기본 true)
    rate_scale      재생 배속. 0.5 면 절반 속도로 천천히 본다 (기본 1.0)
    start_frame     이 프레임부터 재생 (기본 0). 특정 구간만 반복해서 볼 때 쓴다.
                    예: 테스트영상의 방해차량 회피 구간은 frame 2700 부근
    width/height    발행 해상도. 0 이면 원본 그대로 (기본 640x480 — driver_node 의
                    image_width 와 맞춰야 조향 중심이 안 틀어진다)
    rviz            RViz2 도 띄울지 (기본 true)
    auto_start      신호등 없이 바로 LANE_DRIVE 로 시작할지 (기본 true)
    enable          /drive_enable 을 자동으로 켤지 (기본 false)

    (2026-08-19: lidar_confirm 인자는 없어졌다. 회피가 카메라 전용이 되어
     라이다 없는 replay 에서도 실차와 **같은 경로**로 돌아간다 — 예전에는
     이 인자로 트리거 조건을 바꿔야 해서 replay 와 실차 동작이 달랐다.)

⚠️ **화면에 angle=0 speed=0 만 나온다면 십중팔구 이 둘 중 하나다.**

  1) `WAIT_LIGHT  ref[none]` 이면 → FSM 이 초록불을 기다리는 중이다.
     drive_params.yaml 의 `fsm.auto_start` 기본값이 false 라서, 신호등이 없는
     영상에서는 **영원히 출발하지 않는다.** 이 launch 는 벤치 테스트용이므로
     `auto_start` 기본값을 **true 로 뒤집어** 놓았다(실차 launch 는 안 건드린다).
     반대로 신호등 인식/출발 로직 자체를 시험하려면 `auto_start:=false` 로 줄 것.

  2) `LANE_DRIVE ... disabled` 이면 → `/drive_enable` 이 아직 false 다.
     `enable:=true` 를 주거나 다른 터미널에서 직접 켤 것.

⚠️ enable:=true 는 **모터 스택이 떠 있지 않은 벤치에서만** 쓸 것. 이 launch 자체는
   모터를 안 띄우지만, 다른 터미널에 실차 모터 스택이 살아 있으면 차는 영상만 보고
   실제로 달린다. 기본값(false)에서는 driver_node 가 speed 0 을 계속 내므로 판단·
   조향 계산은 전부 돌면서 속도만 0 이다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    video = LaunchConfiguration('video')
    params_file = LaunchConfiguration('params_file')
    overrides_file = LaunchConfiguration('overrides_file')
    use_rviz = LaunchConfiguration('rviz')
    use_enable = LaunchConfiguration('enable')

    args = [
        DeclareLaunchArgument('video', description='재생할 영상 파일 경로 (필수)'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        # 기본값을 params_file 과 같게 둔다 — 같은 파일을 두 번 읽어도 결과가 같아
        # "덮어쓰기 없음"이 된다. 조건부 파라미터를 쓰지 않으려는 의도적 선택.
        DeclareLaunchArgument('overrides_file', default_value=default_params),
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('rate_scale', default_value='1.0'),
        DeclareLaunchArgument('start_frame', default_value='0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # 실차 launch(drive*.launch.py)의 기본값은 그대로 false 다. 여기서만 뒤집는다 —
        # 재생 테스트용 영상에는 보통 신호등이 없어서, false 면 WAIT_LIGHT 에서 멈춘 채
        # 아무것도 안 보인다(그게 "왜 안 움직이지?"의 1번 원인).
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('enable', default_value='false'),
    ]

    common_params = [params_file, overrides_file]

    video_pub = Node(
        package='my_debug',
        executable='video_pub_node',
        name='video_pub_node',
        output='screen',
        parameters=[{
            'video_path': video,
            'loop': LaunchConfiguration('loop'),
            'rate_scale': LaunchConfiguration('rate_scale'),
            'start_frame': LaunchConfiguration('start_frame'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
        }],
    )

    perception = Node(
        package='my_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=common_params + [{
            'model_path': LaunchConfiguration('model'),
            # 재생 테스트는 눈으로 보는 것이 목적이므로 항상 켠다.
            'publish_debug_image': True,
        }],
    )

    driver = Node(
        package='my_driver',
        executable='driver_node',
        name='driver_node',
        output='screen',
        parameters=common_params + [{
            'fsm.auto_start': ParameterValue(
                LaunchConfiguration('auto_start'), value_type=bool),
            # [2026-08-19] 회피가 카메라 전용이 되면서 lateral.require_lidar_confirm
            # 파라미터 자체가 없어졌다. 영상만 있는 replay 에서도 회피 로직이
            # 그대로 돌아가므로 여기서 덮어쓸 것이 없다.
        }],
    )

    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('my_debug'), 'launch', 'viz.launch.py'])),
        launch_arguments={
            'rviz': use_rviz,
            'pipeline_view': 'true',
            'params_file': params_file,
            # 라이다가 없으니 base_link->laser_frame TF 는 의미가 없다.
            'laser_tf': 'false',
        }.items(),
    )

    # 노드들이 올라올 시간을 준 뒤 한 번만 켠다. 위 ⚠️ 경고를 반드시 읽을 것.
    enable = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/drive_enable',
                 'std_msgs/msg/Bool', '{data: true}'],
            output='screen',
        )],
        condition=IfCondition(use_enable),
    )

    return LaunchDescription(args + [video_pub, perception, driver, viz, enable])
