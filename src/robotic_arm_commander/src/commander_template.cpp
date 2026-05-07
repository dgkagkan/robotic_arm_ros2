#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "example_interfaces/msg/bool.hpp"
#include "robotic_arm_interfaces/msg/pose_target.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using PoseTarget = robotic_arm_interfaces::msg::PoseTarget;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using namespace std::placeholders;



class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
        arm_->setMaxVelocityScalingFactor(0.5);
        arm_->setMaxAccelerationScalingFactor(0.5);
        open_gripper_sub_ = node_->create_subscription<Bool>("open_gripper", 10, 
                            std::bind(&Commander::open_gripper_callback, this,_1));
        joint_cmd_sub_ = node->create_subscription<FloatArray>("joint_command", 10,
                            std::bind(&Commander::jointCmd_callback, this, _1));
        cartesian_joint_cmd_sub_ = node->create_subscription<PoseTarget>("cartesion_command",10,
                            std::bind(&Commander::cartesion_callback, this, _1));
    }

    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x , double y , double z , double roll, double pitch,
                                                 double yaw, bool cartesion_path = false)

    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();
        if (!cartesion_path){
        arm_->setPoseTarget(target_pose);
        planAndExecute(arm_);
        }
        else {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);

            if (fraction == 1){
                arm_->execute(trajectory);
            }
        }
    }

    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("open");
        planAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("closed");
        planAndExecute(gripper_);
    }

private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success =  (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success){
            interface->execute(plan);
        }
    }

    void open_gripper_callback(const Bool &msg)
    {
        if (msg.data){
            openGripper();
        }
        else {
            closeGripper();
        }
    }

    void jointCmd_callback(const FloatArray &msg)
    {
        auto joints = msg.data;

        if (joints.size() == 6){
            goToJointTarget(joints);
        }
    }

    void cartesion_callback(const PoseTarget &msg)
    {
        double x = msg.x;
        double y = msg.y;
        double z = msg.z;
        double pitch = msg.pitch;
        double roll = msg.roll;
        double yaw = msg.yaw;
        bool cartesian_path = msg.cartesian_path;
        goToPoseTarget(x, y, z, pitch, roll, yaw, cartesian_path);
    }



    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseTarget>::SharedPtr cartesian_joint_cmd_sub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");

    // Spin σε ξεχωριστό thread ώστε ο MoveGroupInterface να μπορεί να επικοινωνήσει
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    auto commander = Commander(node);

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}