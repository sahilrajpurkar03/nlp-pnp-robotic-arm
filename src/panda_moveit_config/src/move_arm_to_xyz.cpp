#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>
#include <cstdlib> // for std::atof

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: move_arm_to_xyz X Y \n";
        return 1;
    }

    // Parse command-line arguments
    double x = std::atof(argv[1]);
    double y = std::atof(argv[2]);
    double z = 0.2 //std::atof(argv[3]);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_arm_to_xyz_client");

    // Parameter client for the running move_group
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");

    while (!param_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameters...");
    }

    // Get URDF and SRDF from /move_group
    std::string urdf = param_client->get_parameter<std::string>("robot_description");
    std::string srdf = param_client->get_parameter<std::string>("robot_description_semantic");

    // Declare parameters locally for MoveGroupInterface
    node->declare_parameter("robot_description", urdf);
    node->declare_parameter("robot_description_semantic", srdf);

    // Construct MoveGroupInterface for the arm
    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    RCLCPP_INFO(node->get_logger(), "MoveGroupInterface ready for planning.");

    // Set only the position target; orientation is automatically chosen
    move_group.setPositionTarget(x, y, z);

    RCLCPP_INFO(node->get_logger(), "Planning to X: %f, Y: %f, Z: %f", x, y, z);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "Planning successful. Executing...");
        move_group.execute(plan);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }

    rclcpp::shutdown();
    return 0;
}
