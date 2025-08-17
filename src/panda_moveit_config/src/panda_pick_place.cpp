#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>
#include <cstdlib> // for std::atof
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <chrono>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
    auto node = rclcpp::Node::make_shared("panda_pick_place");

    // --- Step 3: Add parameters for Z value and pick height ---
    double z_default = 0.5;
    double pick_height_default = 0.2;
    node->declare_parameter("z_value", z_default);
    node->declare_parameter("pick_height", pick_height_default);

    double z = node->get_parameter("z_value").as_double();
    double pick_height = node->get_parameter("pick_height").as_double();
    RCLCPP_INFO(node->get_logger(), "Initial travel height (z): %f, pick height: %f", z, pick_height);

    auto cb_handle = node->add_on_set_parameters_callback(
        [&z, &pick_height, &node](const std::vector<rclcpp::Parameter> &params) {
            for (auto &p : params) {
                if (p.get_name() == "z_value") {
                    z = p.as_double();
                    RCLCPP_INFO(node->get_logger(), "Updated travel height (z): %f", z);
                } else if (p.get_name() == "pick_height") {
                    pick_height = p.as_double();
                    RCLCPP_INFO(node->get_logger(), "Updated pick height: %f", pick_height);
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

    // Parameter client for move_group
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");

    while (!param_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameters...");
    }

    std::string urdf = param_client->get_parameter<std::string>("robot_description");
    std::string srdf = param_client->get_parameter<std::string>("robot_description_semantic");

    node->declare_parameter("robot_description", urdf);
    node->declare_parameter("robot_description_semantic", srdf);

    // MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface move_group_hand(node, "hand");

    RCLCPP_INFO(node->get_logger(), "MoveGroupInterface ready for planning.");

    // Orientation: downward-facing
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    q.normalize();

    // --- Original joint-value gripper control ---
    auto gripperAction = [&](const std::string &action) {
        double value = 0.0;
        if (action == "open") value = 0.03;
        else if (action == "close") value = 0.001;
        else { RCLCPP_INFO(node->get_logger(), "No such action"); return; }

        move_group_hand.setJointValueTarget("panda_finger_joint1", value);
        move_group_hand.setJointValueTarget("panda_finger_joint2", value);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_hand.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) move_group_hand.execute(plan);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    };

    // --- ROS2 GripperCommand action control ---
    auto gripperActionROS2 = [&node](const std::string &action) {
        using GripperCommand = control_msgs::action::GripperCommand;
        using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

        auto action_client = rclcpp_action::create_client<GripperCommand>(
            node, "panda_hand_controller/gripper_cmd");

        if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
            return;
        }

        double position = (action == "open") ? 0.04 : 0.0;
        double max_effort = 50.0;

        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = max_effort;

        RCLCPP_INFO(node->get_logger(), "Sending ROS2 gripper command: %s", action.c_str());

        auto future_goal_handle = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node, future_goal_handle) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Failed to send gripper goal");
            return;
        }

        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected");
            return;
        }

        auto future_result = action_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node, future_result) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Failed to get gripper result");
        } else {
            RCLCPP_INFO(node->get_logger(),
                        action == "open" ? "Gripper opened via ROS2." : "Gripper closed via ROS2.");
        }
    };

    // --- Move to target XY at travel height ---
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose travel_pose = start_pose;
    travel_pose.position.x = x;
    travel_pose.position.y = y;
    travel_pose.position.z = z;
    travel_pose.orientation.x = q.x();
    travel_pose.orientation.y = q.y();
    travel_pose.orientation.z = q.z();
    travel_pose.orientation.w = q.w();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(travel_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction > 0.99) {
        RCLCPP_INFO(node->get_logger(), "Cartesian path to XY computed successfully. Executing...");
        move_group.execute(trajectory);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed XY path!");
    }

    // --- Open gripper ---
    RCLCPP_INFO(node->get_logger(), "Opening gripper...");
    gripperAction("open");        // Original joint-value control
    // gripperActionROS2("open");  // ROS2 action control (alternative)

    // --- Move down to pick height ---
    geometry_msgs::msg::Pose pick_pose = travel_pose;
    pick_pose.position.z = pick_height;
    waypoints.clear();
    waypoints.push_back(pick_pose);

    fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction > 0.99) {
        RCLCPP_INFO(node->get_logger(), "Moving down to pick height. Executing...");
        move_group.execute(trajectory);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed move down!");
    }

    // --- Close gripper ---
    RCLCPP_INFO(node->get_logger(), "Closing gripper...");
    gripperAction("close");        // Original joint-value control
    // gripperActionROS2("close");  // ROS2 action control (alternative)

    // --- Lift back to travel height ---
    waypoints.clear();
    waypoints.push_back(travel_pose);

    fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction > 0.99) {
        RCLCPP_INFO(node->get_logger(), "Lifting back to travel height. Executing...");
        move_group.execute(trajectory);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed lift up!");
    }

    rclcpp::shutdown();
    return 0;
}
