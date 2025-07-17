from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motion_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config',
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EnderMandS',
    maintainer_email='endermands@qq.com',
    description='cuRobo motion planner ROS2 package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'motion_planner = motion_planner.motion_planner:main',
        ],
    },
)
