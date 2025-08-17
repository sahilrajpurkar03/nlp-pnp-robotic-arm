// file: move_gripper.cpp

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <cstdlib>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter_client.hpp>

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: move_gripper <open|close>\n";
        return 1;
    }

    std::string action = argv[1];
    if (action != "open" && action != "close") {
        std::cerr << "Invalid argument. Use 'open' or 'close'.\n";
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_gripper_client");

    // --- Load URDF/SRDF like original code ---
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");

    while (!param_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameters...");
    }

    std::string urdf = param_client->get_parameter<std::string>("robot_description");
    std::string srdf = param_client->get_parameter<std::string>("robot_description_semantic");

    node->declare_parameter("robot_description", urdf);
    node->declare_parameter("robot_description_semantic", srdf);

    RCLCPP_INFO(node->get_logger(), "URDF and SRDF loaded successfully.");

    // --- Send GripperCommand to ROS 2 controller ---
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

    auto action_client = rclcpp_action::create_client<GripperCommand>(
        node, "panda_hand_controller/gripper_cmd");

    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
        return 1;
    }

    double position = (action == "open") ? 0.04 : 0.0; // open or closed
    double max_effort = 50.0; // adjust as needed

    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = max_effort;

    RCLCPP_INFO(node->get_logger(), "Sending gripper command: %s", action.c_str());

    auto future_goal_handle = action_client->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node, future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send gripper goal");
        rclcpp::shutdown();
        return 1;
    }

    GoalHandleGripper::SharedPtr goal_handle = future_goal_handle.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Gripper goal was rejected");
        rclcpp::shutdown();
        return 1;
    }

    auto future_result = action_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get gripper result");
    } else {
        RCLCPP_INFO(node->get_logger(),
                    action == "open" ? "Gripper opened." : "Gripper closed.");
    }

    rclcpp::shutdown();
    return 0;
}
