#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>
#include <string>
#include <cstdlib>

std::mutex joint_state_mutex;
sensor_msgs::msg::JointState latest_joint_state;
bool joint_state_received = false;

// -------------------- Joint state callback --------------------
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex);
    latest_joint_state = *msg;
    joint_state_received = true;
}

// -------------------- FK helper --------------------
geometry_msgs::msg::Pose getCurrentPoseFK(
    moveit::core::RobotStatePtr kinematic_state,
    const moveit::core::JointModelGroup* joint_model_group)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex);

    geometry_msgs::msg::Pose pose;
    if (!joint_state_received)
        return pose;  // zero pose if joint states not received yet

    std::vector<double> joint_positions;
    for (const auto &name : joint_model_group->getVariableNames())
    {
        auto it = std::find(latest_joint_state.name.begin(), latest_joint_state.name.end(), name);
        if (it != latest_joint_state.name.end())
        {
            size_t index = std::distance(latest_joint_state.name.begin(), it);
            joint_positions.push_back(latest_joint_state.position[index]);
        }
    }
    kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);

    const Eigen::Isometry3d& ee_transform = kinematic_state->getGlobalLinkTransform(
        joint_model_group->getLinkModelNames().back());

    Eigen::Vector3d position = ee_transform.translation();
    Eigen::Quaterniond orientation(ee_transform.rotation());

    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();

    return pose;
}

// -------------------- Fixed-Z wrapper with logging --------------------
bool move_to(double x, double y, double z,
             double roll, double pitch, double yaw,
             moveit::planning_interface::MoveGroupInterface &move_group,
             const std::string &step_description)
{
    RCLCPP_INFO(rclcpp::get_logger("pick_place"), "%s", step_description.c_str());

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

    move_group.setPlanningTime(10.0);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.05);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group.execute(plan);
        return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("pick_place"), "Failed to plan to target pose");
    return false;
}

// -------------------- Grasp feedback --------------------
bool check_grasp_success()
{
    std::lock_guard<std::mutex> lock(joint_state_mutex);

    if (!joint_state_received)
    {
        RCLCPP_WARN(rclcpp::get_logger("grasp"), "No joint states yet, cannot check grasp.");
        return false;
    }

    auto it1 = std::find(latest_joint_state.name.begin(), latest_joint_state.name.end(), "panda_finger_joint1");
    auto it2 = std::find(latest_joint_state.name.begin(), latest_joint_state.name.end(), "panda_finger_joint2");

    if (it1 != latest_joint_state.name.end() && it2 != latest_joint_state.name.end())
    {
        size_t idx1 = std::distance(latest_joint_state.name.begin(), it1);
        size_t idx2 = std::distance(latest_joint_state.name.begin(), it2);

        double pos1 = latest_joint_state.position[idx1];
        double pos2 = latest_joint_state.position[idx2];

        if (pos1 <= 0.001 && pos2 <= 0.001)
        {
            RCLCPP_WARN(rclcpp::get_logger("grasp"), "❌ Grasp likely failed (both fingers fully closed).");
            return false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("grasp"), "✅ Object grasped successfully.");
            return true;
        }
    }

    RCLCPP_ERROR(rclcpp::get_logger("grasp"), "Finger joints not found in joint states.");
    return false;
}

// -------------------- Pick-and-place workflow --------------------
void pick_and_place(double x_pick, double y_pick,
                    double x_drop, double y_drop,
                    moveit::planning_interface::MoveGroupInterface &move_group,
                    moveit::planning_interface::MoveGroupInterface &hand_group,
                    moveit::core::RobotStatePtr kinematic_state,
                    const moveit::core::JointModelGroup* joint_model_group,
                    double approach_z, double pick_z, double carry_z,
                    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub)
{
    std_msgs::msg::Bool msg;
    msg.data = false;
    status_pub->publish(msg);

    geometry_msgs::msg::Pose home_pose = getCurrentPoseFK(kinematic_state, joint_model_group);

    move_to(x_pick, y_pick, approach_z, M_PI, 0, 0, move_group, "Approaching pick position...");
    hand_group.setJointValueTarget("panda_finger_joint1", 0.03);
    hand_group.move();

    move_to(x_pick, y_pick, pick_z, M_PI, 0, 0, move_group, "Moving down to pick object...");
    hand_group.setJointValueTarget("panda_finger_joint1", 0.001);
    hand_group.move();

    if (!check_grasp_success())
    {
        move_to(x_pick, y_pick, pick_z - 0.01, M_PI, 0, 0, move_group, "Retrying pick with lower Z...");
        hand_group.setJointValueTarget("panda_finger_joint1", 0.001);
        hand_group.move();
    }

    move_to(x_pick, y_pick, carry_z, M_PI, 0, 0, move_group, "Lifting object to carrying height...");
    move_to(x_drop, y_drop, carry_z, M_PI, 0, 0, move_group, "Moving object to drop position...");
    move_to(x_drop, y_drop, pick_z, M_PI, 0, 0, move_group, "Lowering object for drop...");
    hand_group.setJointValueTarget("panda_finger_joint1", 0.03);
    hand_group.move();
    move_to(x_drop, y_drop, carry_z, M_PI, 0, 0, move_group, "Moving up after drop...");
    home_pose.position.z += 0.05;
    move_group.setPoseTarget(home_pose);
    move_group.move();
    move_group.setNamedTarget("ready");
    move_group.move();

    msg.data = true;
    status_pub->publish(msg);
}

// -------------------- Main --------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("panda_pick_place");

    double approach_z = 0.23;
    double pick_z = 0.12;
    double carry_z = 0.3;
    double init_angle = -0.3825;

    auto sub_joint = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, jointStateCallback);

    auto status_pub = node->create_publisher<std_msgs::msg::Bool>("/robot_status", 10);

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
        RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameters...");

    std::string urdf = param_client->get_parameter<std::string>("robot_description");
    std::string srdf = param_client->get_parameter<std::string>("robot_description_semantic");
    node->declare_parameter("robot_description", urdf);
    node->declare_parameter("robot_description_semantic", srdf);
    RCLCPP_INFO(node->get_logger(), "URDF and SRDF loaded successfully.");

    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand_group(node, "hand");

    move_group.setPlanningTime(10.0);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.05);

    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader->getModel();
    auto kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    for (int i = 0; i < 20; ++i)
    {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // -------------------- Subscriber for target points --------------------
    auto sub_target = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_point", 10,
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            double x = msg->data[0];
            double y = msg->data[1];
            double yaw_offset = msg->data[2];

            pick_and_place(x, y, -0.3, -0.3, move_group, hand_group,
                           kinematic_state, joint_model_group,
                           approach_z, pick_z, carry_z, status_pub);
        });

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
