import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_debug'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='주행 시각화 (카메라 뷰어 + RViz2 피더) + 영상 재생 테스트',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_view_node = my_debug.pipeline_view_node:main',
            'viz_node = my_debug.viz_node:main',
            'video_pub_node = my_debug.video_pub_node:main',
        ],
    },
)
