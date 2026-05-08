#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotic_arm_interfaces/action/controller.hpp"
#include <thread>
#include <termios.h>
#include <unistd.h>

using namespace std::placeholders;
using Controller = robotic_arm_interfaces::action::Controller;
using ControllerGoalHandle = rclcpp_action::ClientGoalHandle<Controller>;

class TeleopKeyActionClientNode : public rclcpp::Node
{
public:
    TeleopKeyActionClientNode() : Node("teleop_key_action_client")
    {
        controller_client_ = rclcpp_action::create_client<Controller>(this, "controller");
        input_thread_ = std::thread(&TeleopKeyActionClientNode::readInput, this);
        RCLCPP_INFO(this->get_logger(), "Teleop started. R/B/G to pick,C to cancel, Q to quit.");
    }

    ~TeleopKeyActionClientNode()
    {
        if (input_thread_.joinable())
            input_thread_.join();
    }

private:
    char getKey()
    {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void readInput()
    {
        while (rclcpp::ok())
        {
            char key = getKey();
            std::string color;
            switch (key)
            {
                case 'r': color = "red"; break;
                case 'b': color = "blue"; break;
                case 'g': color = "green"; break;
                case 'c':
                    RCLCPP_INFO(this->get_logger(), "Canceling goal...");
                    cancel_goal();
                    continue;
                case 'q':
                    RCLCPP_INFO(this->get_logger(), "Quitting...");
                    restoreTerminal();
                    rclcpp::shutdown();
                    return;
                default:
                    continue;
            }
            RCLCPP_INFO(this->get_logger(), "Sending goal for color: %s", color.c_str());
            send_goal(color);
        }
    }

    void cancel_goal()
    {
        if (goal_handle_) {
            controller_client_->async_cancel_goal(goal_handle_);
            RCLCPP_INFO(this->get_logger(), "Cancel request sent");
        } else {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
        }
    }

    void send_goal(std::string color)
    {
        if (!controller_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal = std::make_shared<Controller::Goal>();
        goal->color = color;

        auto options = rclcpp_action::Client<Controller>::SendGoalOptions();
        options.goal_response_callback = std::bind(&TeleopKeyActionClientNode::goal_response_callback, this, _1);
        options.result_callback = std::bind(&TeleopKeyActionClientNode::goal_result_callback, this, _1);
        options.feedback_callback = std::bind(&TeleopKeyActionClientNode::goal_feedback_callback, this, _1, _2);

        controller_client_->async_send_goal(*goal, options);
    }

    void goal_response_callback(const ControllerGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal was accepted");
            goal_handle_ = goal_handle;
        }
    }

    void goal_result_callback(const ControllerGoalHandle::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Succeeded - x: %f, y: %f",
                    result.result->result_x, result.result->result_y);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result");
                break;
        }
    }

    void goal_feedback_callback(const ControllerGoalHandle::SharedPtr &goal_handle,
                                const std::shared_ptr<const Controller::Feedback> feedback)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Current_x: %f, Current_y: %f",
            feedback->current_x, feedback->current_y);
    }

    void restoreTerminal()
    {
        struct termios oldt;
        tcgetattr(STDIN_FILENO, &oldt);
        oldt.c_lflag |= (ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    std::thread input_thread_;
    ControllerGoalHandle::SharedPtr goal_handle_;
    rclcpp_action::Client<Controller>::SharedPtr controller_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyActionClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}