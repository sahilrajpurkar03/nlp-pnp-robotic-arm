#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>
#include <cstdlib> // for std::atof
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

    // --- Set downward-facing orientation ---
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0); // Roll = 180°, Pitch = 0, Yaw = 0
    q.normalize();

    // --- Define target pose in XY plane ---
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z; // fixed height
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    // --- Compute Cartesian path ---
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(
        waypoints,
        0.01,   // eef_step in meters
        0.0,    // jump_threshold
        trajectory
    );

    if (fraction > 0.99) {
        RCLCPP_INFO(node->get_logger(), "Cartesian path computed successfully. Executing...");
        move_group.execute(trajectory);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to compute Cartesian path! Fraction: %f", fraction);
    }

    rclcpp::shutdown();
    return 0;
}
