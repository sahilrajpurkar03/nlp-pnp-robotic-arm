import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    robot_description = get_package_share_directory("arm_description")

    model_file_path = os.path.join(
        robot_description, "urdf", "panda.urdf"
    )

    # camera_file_path = os.path.join(
    #     get_package_share_directory('robot_description'),
    #     'urdf',
    #     'camera.urdf.xacro'
    # )

    # camera_model_arg = DeclareLaunchArgument(name="camera_model", default_value=os.path.join(
    #                                     robot_description, "urdf", "camera.urdf.xacro"
    #                                     ),
    #                                   description="Absolute path to camera urdf file"
    # )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(robot_description).parent.resolve())
            ]
        )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # --- Instead of running xacro/cat, read the URDF directly ---
    with open(model_file_path, "r") as infp:
        robot_description_content = infp.read()

    # camera_description_param = ParameterValue(Command([
    #         "xacro ",
    #         LaunchConfiguration("camera_model"),
    #         " is_ignition:=",
    #         is_ignition
    #     ]),
    #     value_type=str
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content,
                     "use_sim_time": True}]
    )

    # camera_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": camera_description_param,
    #                  "use_sim_time": True}],
    #     remappings=[("/robot_description", "/camera_description")]
    # )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r empty.sdf "]
                    )
                ]
             )

    gz_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "robot"],
    )

    # gz_spawn_camera = Node(
    #     # package="ros_gz_sim",
    #     # executable="create",
    #     # output="screen",
    #     # arguments=["-topic", "camera_description",
    #     #            "-name", "camera"],

    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=['-file', camera_file_path, '-name', 'camera'],
    #     output='screen'
    # )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            #"/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            #"/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",  # Existing clock bridge
        ],
        output="screen"
    )

    return LaunchDescription([
        # model_arg,   # not needed for plain urdf
        #camera_model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        # camera_state_publisher_node,
        gazebo,
        gz_spawn_robot,
        # gz_spawn_camera,
        gz_ros2_bridge
    ])
