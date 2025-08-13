from setuptools import setup
from glob import glob
import os

package_name = 'yolov8obb_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Resource index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # RViz config files (if any)
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for YOLOv8 OBB object detection',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'data_collector = yolov8obb_object_detection.data_collector:main',
            'yolov8_obb_publisher = yolov8obb_object_detection.yolov8_obb_publisher:main',
            'yolov8_obb_subscriber = yolov8obb_object_detection.yolov8_obb_subscriber:main',
        ],
    },
)
