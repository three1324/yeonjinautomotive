import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 원본은 "imu.yaml" 을 가리켰지만 config/ 에 그런 파일이 없어서
    #   [WARNING] Parameter file path is not a file: .../config/imu.yaml
    # 경고와 함께 파라미터가 통째로 무시됐다 (port, 캘리브레이션 범위, 공분산 전부).
    # 실제로 존재하는 파일은 xycar_imu.yaml (razor.yaml 과 내용 동일) 이다.
    config_path = os.path.join(
        get_package_share_directory("xycar_imu"), "config", "xycar_imu.yaml"
    )
        
    imu_node = Node(
        package="xycar_imu", executable="imu_node", output="screen",
        parameters=[config_path]
    )

    return LaunchDescription([imu_node])

