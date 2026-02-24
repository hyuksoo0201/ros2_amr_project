import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hs_topview'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pinky',
    maintainer_email='pinky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'point_move = hs_topview.point_move:main',
            'waypoint_sender = hs_topview.waypoint_sender:main',
            'map_odom_broadcaster = hs_topview.map_odom_broadcaster:main',
            'pid_controller = hs_topview.pid_controller:main',
        ],
    },
)
