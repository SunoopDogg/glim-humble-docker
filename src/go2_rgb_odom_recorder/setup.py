import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'go2_rgb_odom_recorder'

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
    maintainer='SunoopDogg',
    maintainer_email='aswoo55555@gmail.com',
    description='Record RealSense D435i RGB + Go2 map-frame pose (x,y,theta) to rosbag2 '
                'while teleop-driving inside a prebuilt GLIM map.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'pose_from_tf = go2_rgb_odom_recorder.pose_from_tf:main',
        ],
    },
)
