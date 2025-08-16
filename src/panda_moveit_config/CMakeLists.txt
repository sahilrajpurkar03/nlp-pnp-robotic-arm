cmake_minimum_required(VERSION 3.10.2)
project(panda_moveit_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Executables
add_executable(move_arm_to_xyz src/move_arm_to_xyz.cpp)
ament_target_dependencies(move_arm_to_xyz
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(panda_pick_place src/panda_pick_place.cpp)
ament_target_dependencies(panda_pick_place
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
)

# 🔧 Make sure to install them
install(TARGETS
  move_arm_to_xyz
  panda_pick_place
  DESTINATION lib/${PROJECT_NAME})

# Install launch/config if you have them
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
