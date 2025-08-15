cmake_minimum_required(VERSION 3.10.2)
project(panda_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Build the C++ executable
add_executable(move_arm_to_xyz src/move_arm_to_xyz.cpp)

ament_target_dependencies(move_arm_to_xyz
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

# Install the executable
install(TARGETS move_arm_to_xyz
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install setup assistant file (optional)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
