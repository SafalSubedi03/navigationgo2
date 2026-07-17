import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'go2_nav'

setup(
    name=package_name,
    version='0.0.0',
    maintainer='safal',
    maintainer_email='080bei034.safal@pcampus.edu.np',
    packages=find_packages(exclude=['test']),
    data_files=[
       
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Include all config files (.yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Include all map files (.yaml and .pgm layout files)
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Nav2 and RTAB-Map integration package for Unitree Go2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [    
            'go2_sport_bridge = go2_nav.moveapinode:main',
            'odom_broadcast = go2_nav.odomBroadcast:main',
            'restamp_node = go2_nav.restamp_node:main',
            'cameraimg = go2_vision.cameraaccess:main',
            'object_client = follow_object_client:main',
            'object_server= follow_object_server:main',
        ],
    },
)



