from setuptools import find_packages, setup

package_name = 'go2_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='safal',
    maintainer_email='080bei034.safal@pcampus.edu.np',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cameraimg = go2_vision.cameraaccess:main',
            'yolo_detector = go2_vision.yolo_node:main',
        ],
    },
)
