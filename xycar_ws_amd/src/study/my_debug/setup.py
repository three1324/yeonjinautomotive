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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='파이프라인 시각화 뷰어 (YOLO 검출 + 차선 추정 + 판단/제어 상태를 한 창에)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_view_node = my_debug.pipeline_view_node:main',
        ],
    },
)
