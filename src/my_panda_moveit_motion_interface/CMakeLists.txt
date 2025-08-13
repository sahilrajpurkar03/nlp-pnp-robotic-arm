cmake_minimum_required(VERSION 3.8)
project(my_panda_moveit_motion_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(move_panda_predef_pose src/move_panda_predef_pose.cpp)
add_executable(move_panda_custom_joints src/move_panda_custom_joints.cpp)


target_include_directories(move_panda_predef_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(move_panda_custom_joints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_panda_predef_pose PUBLIC  cxx_std_17)  # Require C99 and C++17
target_compile_features(move_panda_custom_joints PUBLIC cxx_std_17)


ament_target_dependencies(
  move_panda_predef_pose
  moveit_ros_planning_interface
  rclcpp
)

ament_target_dependencies(
  move_panda_custom_joints
  moveit_ros_planning_interface
  rclcpp
)

install(TARGETS 
    move_panda_predef_pose
    move_panda_custom_joints
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
