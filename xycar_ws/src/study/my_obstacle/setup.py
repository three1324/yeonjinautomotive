import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_obstacle'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='라바콘 구간 전담 주행 (라이다). 라이다를 쓰는 노드는 이것 하나뿐이다.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rubbercone_node = my_obstacle.rubbercone_node:main',
        ],
    },
)
