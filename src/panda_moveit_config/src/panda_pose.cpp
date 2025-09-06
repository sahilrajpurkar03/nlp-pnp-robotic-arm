#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <mutex>
#include <thread>

class PandaPoseNode
{
public:
    PandaPoseNode(const rclcpp::Node::SharedPtr& node)
        : node_(node), joint_states_received_(false)
    {
        // --- Load URDF and SRDF from /move_group ---
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node_, "/move_group");
        while (!param_client->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_INFO(node_->get_logger(), "Waiting for /move_group parameters...");

        std::string urdf = param_client->get_parameter<std::string>("robot_description");
        std::string srdf = param_client->get_parameter<std::string>("robot_description_semantic");
        node_->declare_parameter("robot_description", urdf);
        node_->declare_parameter("robot_description_semantic", srdf);

        RCLCPP_INFO(node_->get_logger(), "URDF and SRDF loaded successfully.");

        // --- MoveGroupInterface ---
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "arm");

        // --- FK setup ---
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            node_, "robot_description");
        kinematic_model_ = robot_model_loader_->getModel();
        kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        joint_model_group_ = kinematic_model_->getJointModelGroup("arm");

        // --- Subscribe to joint states ---
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/isaac_joint_states", 10,
            std::bind(&PandaPoseNode::jointStateCallback, this, std::placeholders::_1));
    }

    void printCurrentPose()
    {
        // --- Pose from MoveGroupInterface ---
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
        RCLCPP_INFO(node_->get_logger(), "Pose from MoveGroupInterface:");
        RCLCPP_INFO(node_->get_logger(), " Position -> x: %.3f, y: %.3f, z: %.3f",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);
        RCLCPP_INFO(node_->get_logger(), " Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);

        // --- Pose from joint_states (FK) ---
        if (joint_states_received_)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            kinematic_state_->setVariablePositions(last_joint_state_.name, last_joint_state_.position);
            const Eigen::Isometry3d& ee_state =
                kinematic_state_->getGlobalLinkTransform(move_group_->getEndEffectorLink());

            Eigen::Vector3d position = ee_state.translation();
            Eigen::Quaterniond orientation(ee_state.rotation());

            RCLCPP_INFO(node_->get_logger(), "Pose from FK using /joint_states:");
            RCLCPP_INFO(node_->get_logger(), " Position -> x: %.3f, y: %.3f, z: %.3f",
                        position.x(), position.y(), position.z());
            RCLCPP_INFO(node_->get_logger(), " Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
                        orientation.x(), orientation.y(), orientation.z(), orientation.w());
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "No joint_states received yet.");
        }
    }

    void printJointStates()
    {
        if (joint_states_received_)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            RCLCPP_INFO(node_->get_logger(), "Joint states:");
            for (size_t i = 0; i < last_joint_state_.name.size(); ++i)
            {
                RCLCPP_INFO(node_->get_logger(), "  %s: %.3f",
                            last_joint_state_.name[i].c_str(),
                            last_joint_state_.position[i]);
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "No joint_states received yet.");
        }
    }

    std::mutex mutex_;
    sensor_msgs::msg::JointState last_joint_state_;
    bool joint_states_received_;

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_joint_state_ = *msg;
        joint_states_received_ = true;
    }

    // Members
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    std::shared_ptr<moveit::core::RobotState> kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("panda_get_pose");

    // --- Automatically use simulation time ---
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));


    auto panda_pose = std::make_shared<PandaPoseNode>(node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // --- Wait until joint states are received with valid timestamp ---
    RCLCPP_INFO(node->get_logger(), "Waiting for valid joint states...");
    rclcpp::Time start_time = node->now();
    while (rclcpp::ok()) {
        executor.spin_some();
        {
            std::lock_guard<std::mutex> lock(panda_pose->mutex_);
            if (panda_pose->joint_states_received_ &&
                panda_pose->last_joint_state_.header.stamp.sec != 0) {
                break; // Got a valid timestamp
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if ((node->now() - start_time).seconds() > 5.0) {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting for joint states with valid timestamp");
            break;
        }
    }

    panda_pose->printJointStates();
    panda_pose->printCurrentPose();

    rclcpp::shutdown();
    return 0;
}
