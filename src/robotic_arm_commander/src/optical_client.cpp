#include "rclcpp/rclcpp.hpp"
#include "robotic_arm_interfaces/srv/pick_target.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "robotic_arm_interfaces/msg/controller_string.hpp"
#include <termios.h>


using namespace std::placeholders;
using PickTarget = robotic_arm_interfaces::srv::PickTarget;
using ControllerString = robotic_arm_interfaces::msg::ControllerString;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class MoveItCommanderNode 
{
public:
    MoveItCommanderNode(std::shared_ptr<rclcpp::Node> node)  
    {
        node_ = node;
        auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group;
        controller_sub_ = node->create_subscription<ControllerString>("controller", 10, std::bind(&MoveItCommanderNode::controller_callback, this, _1),
                                                                        sub_options);
        optical_client_ = node->create_client<PickTarget>("pick_target",rmw_qos_profile_services_default, callback_group);
        arm_ = std::make_shared<MoveGroupInterface>(node_,"arm");
        gripper_  = std::make_shared<MoveGroupInterface>(node_,"gripper");
        arm_->setMaxVelocityScalingFactor(0.4);
        arm_->setMaxAccelerationScalingFactor(0.4);
        gripper_->setMaxVelocityScalingFactor(0.5);
        gripper_->setMaxAccelerationScalingFactor(0.5);
    }
    void execute_pick(std::string color)
    {
        if (!optical_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_->get_logger(), "Vision Server not available!");
            return;
        }

        auto request = std::make_shared<PickTarget::Request>();
        request->target_color = color;

        RCLCPP_INFO(node_->get_logger(), "Sending request for color: %s", color.c_str());

        // Στέλνουμε το αίτημα ασύγχρονα. 
        // Η λάμδα συνάρτηση [this](...) θα εκτελεστεί ΜΟΝΟ όταν ο Server απαντήσει.
        optical_client_->async_send_request(request, 
            [this](rclcpp::Client<PickTarget>::SharedFuture future) {
                
                auto response = future.get();

                if (response->success) {
                    RCLCPP_INFO(node_->get_logger(), "Response received successfully!");
                    // Καλούμε την επόμενη συνάρτηση για την κίνηση
                    this->found_target(response);
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Vision Server replied: Target NOT found.");
                }
            }
        );
    }
private:


    void controller_callback(const ControllerString::SharedPtr msg)
    {
        execute_pick(msg->msg);
    }

    void found_target(const std::shared_ptr<PickTarget::Response> target) 
    {   
        if(target->success){
            tf2::Quaternion q;
            q.setRPY(target->roll, target->pitch, target->yaw);
            q = q.normalize();

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";
            target_pose.pose.position.x = target->x;
            target_pose.pose.position.y = target->y;
            target_pose.pose.position.z = target->z;
            target_pose.pose.orientation.x = q.x();
            target_pose.pose.orientation.y = q.y();
            target_pose.pose.orientation.z = q.z();
            target_pose.pose.orientation.w = q.w();
            arm_->setStartStateToCurrentState();
            arm_->setPoseTarget(target_pose);
            plan_and_execute(arm_);
            cartesian_path(target);
        }
    }

    void stack_cubes()
    {
        tf2::Quaternion q;
        q.setRPY(3.14, 0.0, 0.0);
        q = q.normalize();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = -0.4;
        target_pose.pose.position.y = -0.5;
        target_pose.pose.position.z = 0.23;
        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        target_pose.pose.orientation.w = q.w();
        arm_->setPoseTarget(target_pose);
        plan_and_execute(arm_);
    }

    void cartesian_path(const std::shared_ptr<PickTarget::Response> target)
    {
        double fraction;
        double target_width = target->grasp_width;
        std::vector<geometry_msgs::msg::Pose> waypoints_down;
        std::vector<geometry_msgs::msg::Pose> waypoints_up;
        moveit_msgs::msg::RobotTrajectory trajectory;
        
        geometry_msgs::msg::Pose pose1 = arm_->getCurrentPose().pose;
        pose1.position.z += -0.09;
        waypoints_down.push_back(pose1);
        fraction = arm_->computeCartesianPath(waypoints_down, 0.01, trajectory);
        
        if(fraction > 0.95){
            arm_->execute(trajectory);
        }

        close_gripper(target_width);
        rclcpp::sleep_for(std::chrono::seconds(1));

        geometry_msgs::msg::Pose pose2 = pose1;
        pose2.position.z += 0.09;
        waypoints_up.push_back(pose2);

        fraction = arm_->computeCartesianPath(waypoints_up, 0.01, trajectory);
        
        if(fraction > 0.95){
            arm_->execute(trajectory);
        }

        stack_cubes();
        close_gripper(0.12);
    }

    void close_gripper(double total_width)
    {
        double finger_pos = 0.06 - total_width/2;
        std::vector<double> joint_values;
        joint_values.push_back(finger_pos);
        joint_values.push_back(-finger_pos);

        gripper_->setJointValueTarget(joint_values);
        plan_and_execute(gripper_);

    }


    void plan_and_execute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            interface->execute(plan);
        }
        else{
            RCLCPP_INFO(node_->get_logger(),"The robot couldnt find a valid path");
        }
    }



    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Client<PickTarget>::SharedPtr  optical_client_;
    rclcpp::Subscription<ControllerString>::SharedPtr controller_sub_;
    

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    

    auto node = std::make_shared<rclcpp::Node>("moveit_commander");

    auto commander = std::make_shared<MoveItCommanderNode>(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "MoveIt Commander is ready and listening to /controller...");
    

    executor.spin();

    rclcpp::shutdown();
    return 0;
}