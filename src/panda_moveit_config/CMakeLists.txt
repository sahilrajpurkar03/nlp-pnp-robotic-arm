cmake_minimum_required(VERSION 3.10.2)
project(panda_moveit_config)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# --- NEW: Find control_msgs for gripper ---
find_package(control_msgs REQUIRED)

install(PROGRAMS
  scripts/arm_control_from_UI.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Executable 1: move_arm_to_xyz ---
add_executable(move_arm_to_xyz src/move_arm_to_xyz.cpp)
ament_target_dependencies(move_arm_to_xyz
   rclcpp
   geometry_msgs
   moveit_core
   moveit_ros_planning_interface
)

# --- Executable 2: move_gripper ---
add_executable(move_gripper src/move_gripper.cpp)
ament_target_dependencies(move_gripper
   rclcpp
   geometry_msgs
   moveit_core
   moveit_ros_planning_interface
   control_msgs   # <-- Added this line
)

# --- Executable 3: panda_pick_place ---
add_executable(panda_pick_place src/panda_pick_place.cpp)
ament_target_dependencies(panda_pick_place
   rclcpp
   std_msgs
   geometry_msgs
   moveit_core
   moveit_ros_planning_interface
   control_msgs 
)

add_executable(panda_pose src/panda_pose.cpp)
ament_target_dependencies(panda_pose
   rclcpp
   std_msgs
   geometry_msgs
   moveit_core
   moveit_ros_planning_interface
   control_msgs 
)

# Install all executables
install(TARGETS
  move_arm_to_xyz
  move_gripper
  panda_pick_place
  panda_pose
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config directories if any
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
