// This program moves the robot's arm to a predefined named pose specified as a command-line argument.
// It uses MoveIt 2's MoveGroupInterface to plan and execute motion to the named pose configured in the robot's SRDF.


#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: move_panda_predef_pose <named_pose>\n"
              << "Example: move_panda_predef_pose home\n";
    return 1;
  }

  std::string named_pose = argv[1];

  auto node = std::make_shared<rclcpp::Node>(
    "move_panda_predef_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("move_panda_predef_pose");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");  // or your group name

  RCLCPP_INFO(logger, "Moving to named pose: %s", named_pose.c_str());

  // Set the target to the named pose from SRDF
  move_group_interface.setNamedTarget(named_pose);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (bool)move_group_interface.plan(plan);

  if (success) {
    RCLCPP_INFO(logger, "Plan successful, executing...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning to named pose '%s' failed!", named_pose.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
