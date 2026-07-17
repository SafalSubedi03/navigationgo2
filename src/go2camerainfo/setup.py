from setuptools import setup
import os
from glob import glob

package_name = 'go2_camera_info'

setup(
    name=package_name,
    version='0.0.1',
    packages=['go2camerainfo'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Safal',
    maintainer_email='subedisafal856@gmail.com',
    description='Publishes calibrated CameraInfo for the Go2 front camera',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_info_publisher = go2camerainfo.cameraInfoPublisher:main',
        ],
    },
)