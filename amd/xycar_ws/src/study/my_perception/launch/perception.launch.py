"""perception_node 단독 실행 (디버깅용).

    ros2 launch my_perception perception.launch.py
    ros2 launch my_perception perception.launch.py params_file:=/경로/내파일.yaml

파라미터는 my_bringup 의 통합 파일(drive_params.yaml)을 단일 출처로 쓴다.
다만 my_bringup 이 아직 빌드되지 않았어도 이 launch 는 동작해야 한다.
그래서 패키지를 못 찾으면 파라미터 없이 노드 기본값으로 실행한다.
(model_path 는 이 패키지 안의 설치 경로로 항상 채워준다)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_params():
    """my_bringup 의 drive_params.yaml 경로. 없으면 빈 문자열."""
    from ament_index_python.packages import PackageNotFoundError

    try:
        return os.path.join(
            get_package_share_directory('my_bringup'), 'config', 'drive_params.yaml')
    except PackageNotFoundError:
        return ''


def generate_launch_description():
    share = get_package_share_directory('my_perception')
    default_model = os.path.join(share, 'models', 'best5.pt')

    def make_node(context, *args, **kwargs):
        params_file = LaunchConfiguration('params_file').perform(context)
        model_path = LaunchConfiguration('model_path').perform(context)
        parameters = ([params_file] if params_file else []) + [{'model_path': model_path}]
        return [Node(
            package='my_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=parameters,
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=_default_params(),
            description='파라미터 yaml. 비우면 노드 기본값 사용'),
        DeclareLaunchArgument(
            'model_path', default_value=default_model,
            description=(
                'YOLO 가중치 경로. 기본은 이식성 있는 best5.pt(PyTorch, GPU 자동 사용). '
                '더 빠른 추론이 필요하면 같은 models/ 안의 best5.engine(TensorRT, '
                'YOLO(...).export(format="engine", device=0, half=True, imgsz=[480,640])로 '
                '이 젯슨에서 직접 생성, 다른 장비로 복사 불가)을 '
                'model_path:=.../best5.engine 로 지정한다.'
            )),
        OpaqueFunction(function=make_node),
    ])
