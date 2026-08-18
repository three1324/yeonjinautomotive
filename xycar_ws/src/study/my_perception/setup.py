import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # YOLO 가중치. 패키지와 함께 설치해야 런타임에 경로를 찾을 수 있다.
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='카메라 인식 (YOLO11n-seg 1회 추론 -> 차선/신호등/객체)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = my_perception.perception_node:main',
        ],
    },
)
