#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <vector>
#include <string>
#include <iostream>

void move_panda_custom_joints(const std::shared_ptr<rclcpp::Node> node, const std::vector<double> &arm_joints, const std::vector<double> &gripper_joints)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // Set arm target
    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joints);

    // Set gripper target if provided
    bool gripper_within_bounds = true;
    if (!gripper_joints.empty())
        gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joints);

    if (!arm_within_bounds || !gripper_within_bounds)
    {
        RCLCPP_WARN(node->get_logger(),
                    "Target joint position(s) were outside limits, clamping to allowed range.");
        return;
    }

    // Plan both motions
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool gripper_plan_success = true; // default true if no gripper motion

    if (!gripper_joints.empty())
        gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute if successful
    if (arm_plan_success && gripper_plan_success)
    {
        RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
        arm_move_group.execute(arm_plan);
        if (!gripper_joints.empty())
            gripper_move_group.execute(gripper_plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed for arm or gripper.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        std::cerr << "Usage: move_panda_custom_joints <arm_joint1> ... <arm_jointN> [gripper_joint1 gripper_joint2]\n";
        return 1;
    }

    // Assume Panda arm has 7 joints, gripper has 2 joints (optional)
    std::vector<double> arm_joints;
    std::vector<double> gripper_joints;
    try
    {
        for (int i = 1; i <= 7 && i < argc; ++i)
            arm_joints.push_back(std::stod(argv[i]));

        if (argc > 8) // If gripper joints are provided
        {
            for (int i = 8; i < argc; ++i)
                gripper_joints.push_back(std::stod(argv[i]));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Invalid joint value provided.\n";
        return 1;
    }

    auto node = rclcpp::Node::make_shared("move_panda_custom_joints");
    move_panda_custom_joints(node, arm_joints, gripper_joints);

    rclcpp::shutdown();
    return 0;
}
