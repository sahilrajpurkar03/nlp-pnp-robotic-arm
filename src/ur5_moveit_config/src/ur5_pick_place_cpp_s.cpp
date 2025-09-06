#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <thread>
#include <cmath>

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

        // 🔎 Print current EE pose
        geometry_msgs::msg::Pose ee_pose = move_group.getCurrentPose().pose;
        RCLCPP_INFO(rclcpp::get_logger("moveToReadyPose"),
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
        RCLCPP_ERROR(rclcpp::get_logger("moveToReadyPose"), "Failed to plan to arm_ready pose");
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

    if (fraction > 0.99)
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

    if (fraction > 0.99)
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

    if (fraction > 0.99)
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
    if (argc != 4)
    {
        std::cerr << "Usage: ur5_xy_yaw <target_x> <target_y> <yaw_rad>\n";
        return 1;
    }

    double target_x = std::atof(argv[1]);
    double target_y = std::atof(argv[2]);
    double yaw = std::atof(argv[3]);  // yaw in radians

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ur5_xy_yaw_approach");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    const std::string PLANNING_GROUP = "arm";  
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    const std::string GRIPPER_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface gripper_group(node, GRIPPER_GROUP);

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // 1️⃣ Move to ready pose
    if (!moveToReadyPose(move_group))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }    

    // 2️⃣ Move EE to absolute (X,Y) in world
    if (!moveToWorldXY(move_group, target_x, target_y))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 3️⃣ Rotate EE yaw
    if (!rotateGripperYaw(move_group, yaw))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 4️⃣ Move EE straight down by 0.1m
    if (!moveDownZ(move_group, 0.228))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 5️⃣ Close gripper
    if (!closeGripper(gripper_group))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 6️⃣ Move EE back up by 0.221m
    if (!moveUpZ(move_group, 0.228))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }    

    // 7️⃣ Rotate robot base by 180° (pi radians)
    if (!rotateBase(move_group, M_PI))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 8️⃣ Move EE straight down again by 0.228m (for placement)
    if (!moveDownZ(move_group, 0.15))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }   
    
    // 9️⃣ Open gripper (place object)
    if (!openGripper(gripper_group))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 10️⃣ Move EE back up by 0.228m (after placing)
    if (!moveUpZ(move_group, 0.15))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // 11️⃣ Move to ready pose
    if (!moveToReadyPose(move_group))
    {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }  


    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
