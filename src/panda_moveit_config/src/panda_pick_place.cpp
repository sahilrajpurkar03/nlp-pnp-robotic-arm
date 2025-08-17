#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>
#include <cstdlib> // for std::atof
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <chrono>
#include <vector>

// -------------------- Fixed-Z convenience wrappers --------------------
bool move_to(double x, double y, double z,
             double roll, double pitch, double yaw,
             moveit::planning_interface::MoveGroupInterface &move_group)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group.execute(plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("move_to"), "Failed to plan to target pose");
        return false;
    }
}

// -------------------- Cartesian helpers --------------------
bool move_cartesian_to(double x, double y, double z,
                       double roll, double pitch, double yaw,
                       moveit::planning_interface::MoveGroupInterface &move_group)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction > 0.99)
    {
        move_group.execute(trajectory);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("move_cartesian_to"), "Failed to compute Cartesian path");
        return false;
    }
}

// -------------------- Helper motions --------------------
bool move_to_xy_at_approach(double x, double y, double yaw,
                            moveit::planning_interface::MoveGroupInterface &move_group, double approach_z)
{
    // Cartesian approach
    return move_cartesian_to(x, y, approach_z, M_PI, 0.0, yaw, move_group);
}

bool move_to_xy_at_pick(double x, double y, double yaw,
                        moveit::planning_interface::MoveGroupInterface &move_group, double pick_z)
{
    return move_cartesian_to(x, y, pick_z, M_PI, 0.0, yaw, move_group);
}

bool move_to_xy_at_carry(double x, double y, double yaw,
                         moveit::planning_interface::MoveGroupInterface &move_group, double carry_z)
{
    return move_to(x, y, carry_z, M_PI, 0.0, yaw, move_group); // joint-space
}

// -------------------- Gripper helpers --------------------
void open_gripper(moveit::planning_interface::MoveGroupInterface &hand_group)
{
    hand_group.setJointValueTarget("panda_finger_joint1", 0.03);
    hand_group.setJointValueTarget("panda_finger_joint2", 0.03);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (hand_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        hand_group.execute(plan);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void close_gripper(moveit::planning_interface::MoveGroupInterface &hand_group)
{
    hand_group.setJointValueTarget("panda_finger_joint1", 0.001);
    hand_group.setJointValueTarget("panda_finger_joint2", 0.001);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (hand_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        hand_group.execute(plan);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// -------------------- Full pick-and-place workflow --------------------
void pick_and_place(double x_pick, double y_pick,
                    double x_drop, double y_drop,
                    moveit::planning_interface::MoveGroupInterface &move_group,
                    moveit::planning_interface::MoveGroupInterface &hand_group,
                    const geometry_msgs::msg::Pose &home_pose,
                    double approach_z, double pick_z, double carry_z)
{
    // Approach pick location (Cartesian)
    move_to_xy_at_approach(x_pick, y_pick, 0.0, move_group, approach_z);

    open_gripper(hand_group);

    // Lower to pick (Cartesian)
    move_to_xy_at_pick(x_pick, y_pick, 0.0, move_group, pick_z);

    close_gripper(hand_group);

    // Lift after pick (joint-space)
    move_to_xy_at_carry(x_pick, y_pick, 0.0, move_group, carry_z);

    // Move to drop location (Cartesian in XY)
    move_to_xy_at_carry(x_drop, y_drop, 0.0, move_group, carry_z);

    // Lower to drop (Cartesian)
    move_to_xy_at_pick(x_drop, y_drop, 0.0, move_group, pick_z);

    open_gripper(hand_group);

    // Lift after drop (joint-space)
    move_to_xy_at_carry(x_drop, y_drop, 0.0, move_group, carry_z);

    // Return to safe pose & ready (joint-space)
    geometry_msgs::msg::Pose safe_pose = move_group.getCurrentPose().pose;
    safe_pose.position.z += 0.05;
    move_group.setPoseTarget(safe_pose);
    move_group.move();

    bool returned = false;
    for (int i = 0; i < 3 && !returned; i++)
    {
        moveit::planning_interface::MoveGroupInterface::Plan ready_plan;
        move_group.setNamedTarget("ready");

        if (move_group.plan(ready_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            if (move_group.execute(ready_plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Returned to 'ready' pose.");
                returned = true;
            }
        }
    }

    if (!returned)
        RCLCPP_ERROR(rclcpp::get_logger("pick_and_place"), "Failed to return to 'ready' pose!");
}

// -------------------- Main --------------------
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: panda_pick_place X Y \n";
        return 1;
    }

    double x = std::atof(argv[1]);
    double y = std::atof(argv[2]);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("panda_pick_place");

    double approach_z_local = 0.5;
    double pick_z_local     = 0.2;
    double carry_z_local    = 0.5;

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
        RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameters...");

    std::string urdf = param_client->get_parameter<std::string>("robot_description");
    std::string srdf = param_client->get_parameter<std::string>("robot_description_semantic");
    node->declare_parameter("robot_description", urdf);
    node->declare_parameter("robot_description_semantic", srdf);

    node->declare_parameter("approach_z", approach_z_local);
    node->declare_parameter("pick_z", pick_z_local);
    node->declare_parameter("carry_z", carry_z_local);

    approach_z_local = node->get_parameter("approach_z").as_double();
    pick_z_local     = node->get_parameter("pick_z").as_double();
    carry_z_local    = node->get_parameter("carry_z").as_double();

    RCLCPP_INFO(node->get_logger(), "Z heights: approach=%f, pick=%f, carry=%f",
                approach_z_local, pick_z_local, carry_z_local);

    auto cb_handle = node->add_on_set_parameters_callback(
        [&approach_z_local, &pick_z_local, &carry_z_local, &node](const std::vector<rclcpp::Parameter> &params) {
            for (auto &p : params)
            {
                if (p.get_name() == "approach_z") approach_z_local = p.as_double();
                else if (p.get_name() == "pick_z") pick_z_local = p.as_double();
                else if (p.get_name() == "carry_z") carry_z_local = p.as_double();
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand_group(node, "hand");

    geometry_msgs::msg::Pose home_pose = move_group.getCurrentPose().pose;

    pick_and_place(x, y, 0.6, -0.4, move_group, hand_group,
                   home_pose, approach_z_local, pick_z_local, carry_z_local);

    rclcpp::shutdown();
    return 0;
}
