// This program moves the robot's arm to a custom joint configuration specified via command-line arguments.
// It parses the joint angle values, sets them as the target for the MoveIt 2 MoveGroupInterface,
// then plans and executes the motion to reach the given joint positions.

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: move_panda_custom_joints <joint1> <joint2> ... <jointN>\n";
    return 1;
  }

  // Parse joint angles
  std::vector<double> joint_values;
  for (int i = 1; i < argc; ++i) {
    try {
      joint_values.push_back(std::stod(argv[i]));
    } catch (const std::exception & e) {
      std::cerr << "Invalid joint value: " << argv[i] << "\n";
      return 1;
    }
  }

  auto node = std::make_shared<rclcpp::Node>(
    "move_panda_custom_joints",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("move_panda_custom_joints");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");  

  RCLCPP_INFO(logger, "Moving to custom joint positions:");

  for (size_t i = 0; i < joint_values.size(); ++i) {
    RCLCPP_INFO(logger, "  joint[%zu] = %f", i, joint_values[i]);
  }

  // Set joint targets
  move_group_interface.setJointValueTarget(joint_values);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (bool)move_group_interface.plan(plan);

  if (success) {
    RCLCPP_INFO(logger, "Plan successful, executing...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
