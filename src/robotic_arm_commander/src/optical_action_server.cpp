#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotic_arm_interfaces/action/controller.hpp"
#include "robotic_arm_interfaces/srv/pick_target.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>

using Controller = robotic_arm_interfaces::action::Controller;
using ControllerGoalHandle = rclcpp_action::ServerGoalHandle<Controller>;
using PickTarget = robotic_arm_interfaces::srv::PickTarget;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;

class OpticalActionServerNode
{
public:
    OpticalActionServerNode(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        goal_queue_thread_ = std::thread(&OpticalActionServerNode::run_goal_queue_thread, this);
        cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        optical_action_server_ = rclcpp_action::create_server<Controller>(
            node_,
            "controller",
            std::bind(&OpticalActionServerNode::goal_call_back, this, _1, _2),
            std::bind(&OpticalActionServerNode::cancel_callback, this, _1),
            std::bind(&OpticalActionServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
            );
        optical_client_ = node->create_client<PickTarget>("pick_target",rclcpp::ServicesQoS(), cb_group_);
        arm_ = std::make_shared<MoveGroupInterface>(node_,"arm");
        gripper_  = std::make_shared<MoveGroupInterface>(node_,"gripper");
        arm_->setMaxVelocityScalingFactor(0.3);
        arm_->setMaxAccelerationScalingFactor(0.3);
        gripper_->setMaxVelocityScalingFactor(0.5);
        gripper_->setMaxAccelerationScalingFactor(0.5);
    }

    ~OpticalActionServerNode()
    {
        goal_queue_thread_.join();
    }

private:

    rclcpp_action::GoalResponse goal_call_back(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Controller::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(node_->get_logger(),"Received a goal");
        if(!colors_.count(goal->color)){
            RCLCPP_INFO(node_->get_logger(),"Rejectring the goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(node_->get_logger(),"Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<ControllerGoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(),"Received a cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<ControllerGoalHandle>goal_handle)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_queue_.push(goal_handle);
        RCLCPP_INFO(node_->get_logger(),"new goal added to queue");
        RCLCPP_INFO(node_->get_logger(),"Queue size: %d", (int)goal_queue_.size());
    }

    void run_goal_queue_thread()
    {
        rclcpp::Rate loop_rate(1000.0);
        while(rclcpp::ok())
        {   
            std::shared_ptr<ControllerGoalHandle> next_goal;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(!busy_ && !goal_queue_.empty())
                {
                    next_goal = goal_queue_.front();
                    goal_queue_.pop();
                    busy_ = true;
                }
            }
            
            if(next_goal)
            {
                RCLCPP_INFO(node_->get_logger(),"Next goal is executing");
                execute_goal(next_goal);
            }
            
            loop_rate.sleep();
        }
    }

    void execute_goal(const std::shared_ptr<ControllerGoalHandle>goal_handle)
    {
        //get request
        std::string color = goal_handle->get_goal()->color;
        auto result = std::make_shared<Controller::Result>();
        //execute goal
        execute_pick(color, goal_handle);
    }

    void execute_pick(std::string color, std::shared_ptr<ControllerGoalHandle> goal_handle)
    {
        if (!optical_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_->get_logger(), "Vision Server not available!");
            return;
        }

        auto request = std::make_shared<PickTarget::Request>();
        request->target_color = color;

        RCLCPP_INFO(node_->get_logger(), "Sending request for color: %s", color.c_str());

        optical_client_->async_send_request(request, 
                [this, goal_handle](rclcpp::Client<PickTarget>::SharedFuture future) {
                    
                    auto response = future.get();

                    if (response->success) {
                        this->found_target(response);

                        auto result = std::make_shared<Controller::Result>();
                        result->result_x = response->x;
                        result->result_y = response->y; 
                        result->msg = "The cube placed at x= " + std::to_string(response->x) 
                                    + ", y= " + std::to_string(response->y);
                        goal_handle->succeed(result);
                    } else {
                        auto result = std::make_shared<Controller::Result>();
                        goal_handle->abort(result);
                    }

                    std::lock_guard<std::mutex> lock(mutex_);
                    if (response->success) {
                        stack_height_ += response->grasp_width+0.015;
                    }
                    busy_ = false;
                }
            );
    }

    void found_target(const std::shared_ptr<PickTarget::Response> target) 
    {   
        if(target->success){
            double grasp_width = target->grasp_width;
            double cube_height = target->grasp_width;

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
            cartesian_down(cartesian_down_);
            open_gripper(grasp_width);
            rclcpp::sleep_for(std::chrono::seconds(1));
            cartesian_up(cartesian_up_);
            stack_cubes(grasp_width, cube_height);
        }
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

    void cartesian_down(double distance)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    geometry_msgs::msg::Pose pose = arm_->getCurrentPose().pose;
    pose.position.z += distance;
    waypoints.push_back(pose);
    
    double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
    if (fraction > 0.95) {
        arm_->execute(trajectory);
    }
}

void cartesian_up(double distance)
{
    cartesian_down(distance);
}

    void stack_cubes(double grasp_width, double cube_height)
    {
        tf2::Quaternion q;
        q.setRPY(3.14, 0.0, 0.0);
        q = q.normalize();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = -0.4;
        target_pose.pose.position.y = -0.5;
        target_pose.pose.position.z = stack_height_;
        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        target_pose.pose.orientation.w = q.w();
        arm_->setPoseTarget(target_pose);
        plan_and_execute(arm_);
        cartesian_down(-cube_height - 0.03);
        rclcpp::sleep_for(std::chrono::seconds(1));
        open_gripper(grasp_width + 0.1);
        cartesian_up(cube_height + 0.03);
    }

    void open_gripper(double total_width)
    {
        double finger_pos = 0.06 - total_width/2;
        std::vector<double> joint_values;
        joint_values.push_back(finger_pos);
        joint_values.push_back(-finger_pos);

        gripper_->setJointValueTarget(joint_values);
        plan_and_execute(gripper_);

    }



    double cartesian_down_ = -0.09;
    double cartesian_up_ = 0.09;
    double stack_height_ = 0.165;
    bool busy_ = false;
    std::shared_ptr<rclcpp::Node> node_;
    std::set<std::string> colors_ = {"blue", "red", "green"};
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    rclcpp_action::Server<Controller>::SharedPtr optical_action_server_;
    rclcpp::Client<PickTarget>::SharedPtr optical_client_;
    std::queue<std::shared_ptr<ControllerGoalHandle>> goal_queue_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::thread goal_queue_thread_;
};





int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("optical_action_server");
    auto server_node = std::make_shared<OpticalActionServerNode>(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}