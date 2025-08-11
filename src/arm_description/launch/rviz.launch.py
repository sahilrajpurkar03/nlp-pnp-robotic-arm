from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your robot urdf file (change this to your file path)
    urdf_file = os.path.join(
        get_package_share_directory('arm_description'),  # replace with your package name
        'urdf',
        'my_panda.urdf'  # replace with your urdf filename
    )
    
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    return LaunchDescription([
        # Robot State Publisher publishes joint states to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        # Joint State Publisher GUI (optional, allows joint manipulation)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        # RViz node to visualize robot
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('arm_description'), 'rviz', 'view_robot.rviz')]  # Optional rviz config file
        ),
    ])
