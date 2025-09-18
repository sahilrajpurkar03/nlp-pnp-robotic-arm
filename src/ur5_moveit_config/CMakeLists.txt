cmake_minimum_required(VERSION 3.22)
project(ur5_moveit_config)

# --- Find packages ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(yolov8_obb_msgs REQUIRED)  

ament_export_dependencies(rosidl_default_runtime)

# --- C++ executables ---
add_executable(ur5_pick_place_cpp_r src/ur5_pick_place_cpp_r.cpp)
ament_target_dependencies(ur5_pick_place_cpp_r
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
  yolov8_obb_msgs
)

add_executable(ur5_pick_place_cpp_s src/ur5_pick_place_cpp_s.cpp)
ament_target_dependencies(ur5_pick_place_cpp_s
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
  yolov8_obb_msgs
)

# --- Install C++ executables ---
install(TARGETS 
  ur5_pick_place_cpp_r
  ur5_pick_place_cpp_s
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install Python scripts (commented) ---
# install(PROGRAMS
#   scripts/ur5_pick_place_r.py
#   scripts/ur5_pick_place_s.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# --- Install launch/config files ---
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
