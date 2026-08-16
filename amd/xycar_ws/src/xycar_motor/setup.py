from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xycar_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 시리얼 포트 점검 스크립트. 소스 트리에서 직접 실행해도 되지만
        # 설치본에서도 찾을 수 있게 함께 배포한다.
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='xycar_motor 토픽을 받아 vesc_ackermann으로 넘기는 모터 구동 노드 (ROS1 이식)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xycar_motor = xycar_motor.xycar_motor_node:main',
        ],
    },
)
