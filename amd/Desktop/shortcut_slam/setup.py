import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'shortcut_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master',
    maintainer_email='biscuit2354@khu.ac.kr',
    description='숏컷 구간 독립 SLAM 매핑 검증 패키지 (rf2o + slam_toolbox)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_check_node = shortcut_slam.scan_check_node:main',
        ],
    },
)
