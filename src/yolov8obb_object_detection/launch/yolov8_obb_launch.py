#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Node to publish YOLOv8 inferences
    yolov8_publisher_node = Node(
        package='yolov8obb_object_detection',
        executable='yolov8_obb_publisher',
        name='yolov8_obb_publisher',
        output='screen'
    )

    # Node to subscribe and draw YOLOv8 inferences
    yolov8_subscriber_node = Node(
        package='yolov8obb_object_detection',
        executable='yolov8_obb_subscriber',
        name='yolov8_obb_subscriber',
        output='screen'
    )

    # RViz2 node to visualize the result
    pkg_share = get_package_share_directory('yolov8obb_object_detection')
    rviz_config_file = os.path.join(pkg_share, 'config', 'yolov8_obb.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        yolov8_publisher_node,
        yolov8_subscriber_node,
        rviz_node
    ])
