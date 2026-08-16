"""driver_node 단독 실행 (디버깅용).

인지 노드를 이미 띄워놓고 제어만 다시 시작할 때 쓴다.

    ros2 launch my_driver driver.launch.py
    ros2 launch my_driver driver.launch.py params_file:=/경로/내파일.yaml

파라미터는 my_bringup 의 통합 파일(drive_params.yaml)을 단일 출처로 쓴다.
다만 my_bringup 이 아직 빌드되지 않았어도 이 launch 는 동작해야 한다.
그래서 패키지를 못 찾으면 파라미터 없이 노드 기본값으로 실행한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_params():
    """my_bringup 의 drive_params.yaml 경로. 없으면 빈 문자열."""
    import os

    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        return os.path.join(
            get_package_share_directory('my_bringup'), 'config', 'drive_params.yaml')
    except PackageNotFoundError:
        return ''


def generate_launch_description():
    def make_node(context, *args, **kwargs):
        params_file = LaunchConfiguration('params_file').perform(context)
        parameters = [params_file] if params_file else []
        return [Node(
            package='my_driver',
            executable='driver_node',
            name='driver_node',
            output='screen',
            parameters=parameters,
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=_default_params(),
            description='파라미터 yaml. 비우면 노드 기본값 사용'),
        OpaqueFunction(function=make_node),
    ])
