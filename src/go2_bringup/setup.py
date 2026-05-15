import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'go2_bringup'


def recursive_data_files(src_dir):
    """Install every file under src_dir, preserving subdirs, into share/<pkg>/src_dir."""
    entries = []
    for path in glob(os.path.join(src_dir, '**', '*'), recursive=True):
        if os.path.isfile(path):
            dest = os.path.join('share', package_name, os.path.dirname(path))
            entries.append((dest, [path]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        *recursive_data_files('config'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SunoopDogg',
    maintainer_email='aswoo55555@gmail.com',
    description='Bringup for Go2 EDU + Ouster OS1-32 real-hardware mapping session.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'go2_sport_bridge = go2_bringup.go2_sport_bridge:main',
        ],
    },
)
