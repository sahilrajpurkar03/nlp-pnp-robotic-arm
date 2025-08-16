#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>
#include <cstdlib> // for std::atof

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: panda_pick_place X Y \n";
        return 1;
    }

    // Parse command-line arguments
    double x = std::atof(argv[1]);
    double y = std::atof(argv[2]);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("panda_pick_place");  // ✅ renamed node

    // --- Step 3: Add parameter for Z value ---
    // z_value represents the "pickup height" of the end-effector above the surface.
    // Default = 0.2 m (20 cm), but can be changed at runtime with:
    //   ros2 param set /panda_pick_place z_value 0.25
    double z_default = 0.5;
    node->declare_parameter("z_value", z_default);

    // Allow runtime changes
    double z = node->get_parameter("z_value").as_double();
    RCLCPP_INFO(node->get_logger(), "Initial pickup height (z): %f", z);

    auto cb_handle = node->add_on_set_parameters_callback(
        [&z, &node](const std::vector<rclcpp::Parameter> &params) {
            for (auto &p : params) {
                if (p.get_name() == "z_value") {
                    z = p.as_double();
                    RCLCPP_INFO(node->get_logger(), "Updated pickup height (z): %f", z);
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

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

    RCLCPP_INFO(node->get_logger(), "Planning to X: %f, Y: %f, Z (pickup height): %f", x, y, z);

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
