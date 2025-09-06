#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <vector>
#include <thread>

// --------------------- Function: Move to ready pose ---------------------
bool moveToReadyPose(moveit::planning_interface::MoveGroupInterface &move_group)
{
    move_group.setStartStateToCurrentState();
    move_group.setNamedTarget("arm_ready");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("moveToReadyPose"), "Moving to arm_ready pose...");
        move_group.execute(plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveToReadyPose"), "Failed to plan to arm_ready pose");
        return false;
    }
}

// --------------------- Function: Move end-effector in local XY plane ---------------------
bool moveInEE_XY(moveit::planning_interface::MoveGroupInterface &move_group,
                 double dx, double dy)
{
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    tf2::Vector3 ee_z(0, 0, -1);
    ee_z.normalize();

    tf2::Vector3 ee_x(1, 0, 0);
    ee_x = ee_x - ee_x.dot(ee_z) * ee_z;
    ee_x.normalize();

    tf2::Vector3 ee_y = ee_z.cross(ee_x);
    ee_y.normalize();

    tf2::Vector3 translation = ee_x * dx + ee_y * dy;

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += translation.x();
    target_pose.position.y += translation.y();
    target_pose.position.z += translation.z();
    target_pose.orientation = current_pose.orientation;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    if (fraction > 0.99)
    {
        move_group.execute(trajectory);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveInEE_XY"), "Failed to compute horizontal XY path. Fraction: %f", fraction);
        return false;
    }
}

// --------------------- Function: Move end-effector in Z ---------------------
bool moveInEE_Z(moveit::planning_interface::MoveGroupInterface &move_group, double dz)
{
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.z += dz;
    target_pose.orientation = current_pose.orientation;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    if (fraction > 0.99)
    {
        move_group.execute(trajectory);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveInEE_Z"), "Failed to compute Z path. Fraction: %f", fraction);
        return false;
    }
}

// --------------------- Function: Move end-effector to arbitrary pose (non-Cartesian) ---------------------
bool moveToPose(moveit::planning_interface::MoveGroupInterface &move_group,
                const geometry_msgs::msg::Pose &target_pose,
                const std::string &log_name = "moveToPose")
{
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger(log_name),
                    "Executing move to pose: x=%.3f, y=%.3f, z=%.3f",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        move_group.execute(plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(log_name), "Failed to plan to target pose!");
        return false;
    }
}

// --------------------- Main ---------------------
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: ur5_xy_approach <dx> <dy>\n";
        return 1;
    }

    double dx = std::atof(argv[1]);
    double dy = std::atof(argv[2]);
    double z_pick = -0.2;
    double z_drop = -0.4;
    double drop_x = 0.2;
    double drop_y = -0.6;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ur5_xy_approach");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // 1️⃣ Move to ready pose
    if (!moveToReadyPose(move_group))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot move to ready pose. Exiting.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 2️⃣ Move in XY plane to pick location
    if (!moveInEE_XY(move_group, dx, dy))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot move in EE XY plane. Exiting.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 3️⃣ Move down for picking
    if (!moveInEE_Z(move_group, z_pick))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot move down to pick. Exiting.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 4️⃣ Move up after picking
    if (!moveInEE_Z(move_group, -z_pick))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot move up. Exiting.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 5️⃣ Move to drop location using normal planning
    geometry_msgs::msg::Pose drop_pose = move_group.getCurrentPose().pose;
    drop_pose.position.x = drop_x;
    drop_pose.position.y = drop_y;
    drop_pose.position.z = 0.391;  // safe drop height
    if (!moveToPose(move_group, drop_pose, "DropXYMove"))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot move to drop location. Exiting.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 6️⃣ Move down to drop
    if (!moveInEE_Z(move_group, z_drop))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot move down to drop. Exiting.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 7️⃣ Move up after drop
    if (!moveInEE_Z(move_group, -z_drop))
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot move up after drop. Exiting.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 8️⃣ Return to ready pose using normal planning
    move_group.setPlanningTime(10.0);      // allow longer planning
    move_group.setNumPlanningAttempts(10); // try multiple times
    if (!moveToReadyPose(move_group))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to return to arm_ready pose.");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Returned to arm_ready pose successfully.");
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
