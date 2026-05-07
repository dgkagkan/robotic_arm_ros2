#include "rclcpp/rclcpp.hpp"
#include "robotic_arm_interfaces/msg/controller_string.hpp"
#include <thread>
#include <iostream>
#include <termios.h>
#include <unistd.h>
    
using ControllerString = robotic_arm_interfaces::msg::ControllerString;


class TeleopKeyboardNode : public rclcpp::Node
{
public:
    TeleopKeyboardNode() : Node("teleop_keyboard") 
    {
        controller_pub_ = this->create_publisher<ControllerString>("controller",10);
        input_thread_ = std::thread(&TeleopKeyboardNode::readInput, this);
        RCLCPP_INFO(this->get_logger(), "Teleop started. W/A/S/D to move, Q to quit.");
    }
    
    
    ~TeleopKeyboardNode()
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


    void publishcontroller()
    {
        auto msg = ControllerString();
        msg.msg = color_;
        controller_pub_->publish(msg);
    }

    void readInput()
    {
        while(rclcpp::ok())
        {
            char key = getKey();
            switch(key)
            {
                case 'r': color_ ="red"; break;
                case 'b': color_ ="blue"; break;
                case 'g': color_ ="green"; break;
                case 'q':
                    RCLCPP_INFO(this->get_logger(),"Quiting...");
                    restoreTerminal();
                    rclcpp::shutdown();
                    return;
                default:
                    color_ = "";
            }
            RCLCPP_INFO(this->get_logger(), "Color: %s", color_.c_str());
            publishcontroller();
        }
    }

    void restoreTerminal()
    {
        struct termios oldt;
        tcgetattr(STDIN_FILENO, &oldt);
        oldt.c_lflag |= (ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }



    std::string color_;
    std::thread input_thread_;
    rclcpp::Publisher<ControllerString>::SharedPtr controller_pub_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboardNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}