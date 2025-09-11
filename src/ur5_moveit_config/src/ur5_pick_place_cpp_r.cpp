#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <thread>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mutex>


// --------------------- Function: Move to ready pose ---------------------
bool moveToNamedPose(moveit::planning_interface::MoveGroupInterface &move_group,
                     const std::string &target_name)
{
    move_group.setStartStateToCurrentState();
    move_group.setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("moveToNamedPose"), "Moving to '%s' pose...", target_name.c_str());
        move_group.execute(plan);

        // 🔎 Print current EE pose
        geometry_msgs::msg::Pose ee_pose = move_group.getCurrentPose().pose;
        RCLCPP_INFO(rclcpp::get_logger("moveToNamedPose"),
                    "EE Pose in world (base_link):\n  Position -> x: %.3f, y: %.3f, z: %.3f\n  Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
                    ee_pose.position.x,
                    ee_pose.position.y,
                    ee_pose.position.z,
                    ee_pose.orientation.x,
                    ee_pose.orientation.y,
                    ee_pose.orientation.z,
                    ee_pose.orientation.w);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveToNamedPose"), "Failed to plan to '%s' pose", target_name.c_str());
        return false;
    }
}

// --------------------- Function: Move EE to absolute (X,Y) in world ---------------------
bool moveToWorldXY(moveit::planning_interface::MoveGroupInterface &move_group,
                   double target_x, double target_y)
{
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x = target_x;
    target_pose.position.y = target_y;

    RCLCPP_INFO(rclcpp::get_logger("moveToWorldXY"),
                "Target Pose in world:\n  Position -> x: %.3f, y: %.3f, z: %.3f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    if (fraction > 0.85)
    {
        move_group.execute(trajectory);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveToWorldXY"),
                     "Failed to compute Cartesian path. Fraction: %f", fraction);
        return false;
    }
}

// --------------------- Function: Rotate EE yaw ---------------------
bool rotateGripperYaw(moveit::planning_interface::MoveGroupInterface &move_group,
                      double yaw)
{
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    // Convert current orientation to roll-pitch-yaw
    tf2::Quaternion q_orig, q_rot;
    tf2::fromMsg(current_pose.orientation, q_orig);

    double roll, pitch, cur_yaw;
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, cur_yaw);

    // Build new quaternion with given yaw, keep roll/pitch
    q_rot.setRPY(roll, pitch, yaw);
    q_rot.normalize();

    geometry_msgs::msg::Pose target_pose = current_pose;
    geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q_rot);
    target_pose.orientation = q_msg;


    RCLCPP_INFO(rclcpp::get_logger("rotateGripperYaw"),
                "Rotating gripper to yaw=%.3f rad (%.1f deg)", yaw, yaw * 180.0 / M_PI);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction > 0.85)
    {
        move_group.execute(trajectory);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rotateGripperYaw"),
                     "Failed to compute yaw rotation path. Fraction: %f", fraction);
        return false;
    }
}

// --------------------- Function: Move EE downward in Z ---------------------
bool moveDownZ(moveit::planning_interface::MoveGroupInterface &move_group,
               double dz)
{
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.z -= dz;  // go down by dz

    RCLCPP_INFO(rclcpp::get_logger("moveDownZ"),
                "Target Pose going down:\n  Position -> x: %.3f, y: %.3f, z: %.3f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    if (fraction > 0.85)
    {
        move_group.execute(trajectory);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveDownZ"),
                     "Failed to compute Z downward path. Fraction: %f", fraction);
        return false;
    }
}

// --------------------- Function: Close gripper using MoveIt named target ---------------------
bool closeGripper(moveit::planning_interface::MoveGroupInterface &gripper_group)
{
    gripper_group.setStartStateToCurrentState();
    gripper_group.setNamedTarget("close");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (gripper_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("closeGripper"), "Closing gripper (named target: close)...");
        gripper_group.execute(plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("closeGripper"), "Failed to plan to gripper close pose");
        return false;
    }
}

// --------------------- Function: Move EE upward in Z ---------------------
bool moveUpZ(moveit::planning_interface::MoveGroupInterface &move_group,
             double dz)
{
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.z += dz;  // go up by dz

    RCLCPP_INFO(rclcpp::get_logger("moveUpZ"),
                "Target Pose going up:\n  Position -> x: %.3f, y: %.3f, z: %.3f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    if (fraction > 0.85)
    {
        move_group.execute(trajectory);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveUpZ"),
                     "Failed to compute Z upward path. Fraction: %f", fraction);
        return false;
    }
}

// --------------------- Function: Rotate base joint ---------------------
bool rotateBase(moveit::planning_interface::MoveGroupInterface &move_group,
                double delta_angle)
{
    // Get current joint values
    std::vector<double> joint_group_positions = move_group.getCurrentJointValues();

    // Joint[0] = base rotation (shoulder_pan_joint)
    double current_angle = joint_group_positions[0];
    joint_group_positions[0] = current_angle + delta_angle;  // add delta (e.g., M_PI for 180°)

    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rotateBase"),
                    "Rotating base joint by %.3f rad (%.1f deg)",
                    delta_angle, delta_angle * 180.0 / M_PI);
        move_group.execute(plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rotateBase"),
                     "Failed to plan base rotation");
        return false;
    }
}

// --------------------- Function: Open gripper using MoveIt named target ---------------------
bool openGripper(moveit::planning_interface::MoveGroupInterface &gripper_group)
{
    gripper_group.setStartStateToCurrentState();
    gripper_group.setNamedTarget("open");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (gripper_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("openGripper"), "Opening gripper (named target: open)...");
        gripper_group.execute(plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("openGripper"), "Failed to plan to gripper open pose");
        return false;
    }
}


// --------------------- Main ---------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ur5_xy_yaw_approach");

    const std::string PLANNING_GROUP = "arm";  
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    const std::string GRIPPER_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface gripper_group(node, GRIPPER_GROUP);

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // ---- Shared target variables ----
    std::mutex mtx;
    bool new_target_available = false;
    double target_x = 0.0, target_y = 0.0, target_yaw = 0.0;

    // ---- Subscriber ----
    auto sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_point", 10,
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() < 3)
            {
                RCLCPP_ERROR(node->get_logger(), "Received target_point with insufficient data.");
                return;
            }

            std::lock_guard<std::mutex> lock(mtx);
            target_x = std::round(msg->data[0] * 10.0) / 10.0;
            target_y = std::round(msg->data[1] * 10.0) / 10.0;
            target_yaw = std::round(msg->data[2] * 10.0) / 10.0;
            new_target_available = true;

            RCLCPP_INFO(node->get_logger(),
                        "Received target_point -> x=%.1f, y=%.1f, yaw=%.1f",
                        target_x, target_y, target_yaw);
        });

    // ---- Executor and spinner ----
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    // ---- Main loop to execute MoveIt motions ----
    while (rclcpp::ok())
    {
        bool run_motion = false;
        double x, y, yaw;
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (new_target_available)
            {
                x = target_x; y = target_y; yaw = target_yaw;
                new_target_available = false;
                run_motion = true;
            }
        }

        if (run_motion)
        {
            //if (!moveToNamedPose(move_group, "arm_ready_1")) { break; }
            //rclcpp::sleep_for(std::chrono::seconds(3));
            long int delay = 1;

            if (!moveToWorldXY(move_group, x, y)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!rotateGripperYaw(move_group, yaw)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!moveDownZ(move_group, 0.228)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!closeGripper(gripper_group)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!moveUpZ(move_group, 0.228)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!rotateBase(move_group, M_PI)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!moveDownZ(move_group, 0.15)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!openGripper(gripper_group)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!moveUpZ(move_group, 0.15)) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            if (!moveToNamedPose(move_group, "arm_ready")) { break; }
            rclcpp::sleep_for(std::chrono::seconds(delay));

            RCLCPP_INFO(node->get_logger(), "✅ Completed pick & place cycle.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
