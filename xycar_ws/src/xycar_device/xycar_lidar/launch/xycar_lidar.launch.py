#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    share_dir = get_package_share_directory('xycar_lidar')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'xycar_lidar_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    # ⚠️ 원래 여기는 LifecycleNode 액션이었다. 그런데 src/xycar_lidar_node.cpp 는
    #    rclcpp::Node::make_shared() 로 만든 **평범한 노드**다 (LifecycleNode 아님).
    #    lifecycle 액션으로 띄우면 configure/activate 전이가 영영 안 오므로
    #    "라이다가 안 켜졌다"고 오해하기 쉽다. 실제로는 프로세스가 그냥 떠서
    #    /scan 을 내지만, 상태 조회(ros2 lifecycle get)는 실패한다.
    #    2026-08-19 실차 확인: 이 노드로 바꿔도 /scan 9.66Hz 정상.
    driver_node = Node(package='xycar_lidar',
                       executable='xycar_lidar_node',
                       name='xycar_lidar_node',
                       output='screen',
                       emulate_tty=True,
                       parameters=[parameter_file],
                       )

    return LaunchDescription([
        params_declare,
        driver_node,
    ])
