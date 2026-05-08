#include "rclcpp/rclcpp.hpp"
#include "robotic_arm_interfaces/srv/pick_target.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include <cmath>
    
using namespace std::placeholders;
using PickTarget = robotic_arm_interfaces::srv::PickTarget;
using Image = sensor_msgs::msg::Image;

class PickTargetServerNode : public rclcpp::Node 
{
public:
    PickTargetServerNode() : Node("optical_server") 
    {

        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;

        // Subscriber - λαμβάνει εικόνα από κάμερα
        image_sub_ = this->create_subscription<Image>(
            "/camera/image_raw",
            10,
            std::bind(&PickTargetServerNode::camera_callback, this, _1),
            sub_options
        );

        // Publisher - δημοσιεύει debug εικόνα με bounding boxes
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/debug_image", 
            rclcpp::SensorDataQoS()
        );

        // Service server - επιστρέφει θέση κύβου βάσει χρώματος
        optical_server_ = this->create_service<PickTarget>(
            "pick_target",
            std::bind(&PickTargetServerNode::callbackPickTarget, this, _1, _2),
            rmw_qos_profile_services_default,
            callback_group_
        );
    }
    
private:

    // Επεξεργασία κάθε frame - ανίχνευση χρωμάτων σε HSV
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat hsv_frame;
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

            // Μπλε μάσκα
            cv::Mat blue_mask;
            cv::inRange(hsv_frame, cv::Scalar(100, 100, 100), cv::Scalar(130, 255, 255), blue_mask);
            process_color(blue_mask, frame, blue_x_, blue_y_, blue_yaw_, "blue");

            // Κόκκινη μάσκα (δύο εύρη γιατί το κόκκινο τυλίγεται στο HSV)
            cv::Mat mask1, mask2, red_mask;
            cv::inRange(hsv_frame, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(hsv_frame, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);
            cv::addWeighted(mask1, 1.0, mask2, 1.0, 0.0, red_mask);
            process_color(red_mask, frame, red_x_, red_y_, red_yaw_, "red");

            // Πράσινη μάσκα
            cv::Mat green_mask;
            cv::inRange(hsv_frame, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), green_mask);
            process_color(green_mask, frame, green_x_, green_y_, green_yaw_, "green");

            // Δημοσίευση debug εικόνας
            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            debug_pub_->publish(*debug_msg);

            
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        }
    }

    // Μετατροπή pixel σε μέτρα (pinhole camera model) και εύρεση yaw
    void process_color(cv::Mat mask, cv::Mat &output_frame, double &target_x, double &target_y, double &target_yaw, std::string color_name) 
    {
        // Camera intrinsics
        const double fx = 381.36116;
        const double fy = 381.36114;
        const double cx_center = 320.0;
        const double cy_center = 240.0;
        const double Z_dist = 2.0;
        const double y_offset = -0.7;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            if (cv::contourArea(contour) > 5) { 
                cv::Rect bbox = cv::boundingRect(contour);
                int cx = bbox.x + (bbox.width / 2);
                int cy = bbox.y + (bbox.height / 2);
                cv::RotatedRect rotated_bbox = cv::minAreaRect(contour);
                cv::Point2f center = rotated_bbox.center;
                float angle = rotated_bbox.angle;

                // Yaw από rotated bounding box
                double yaw_rad = angle * (M_PI / 180.0);
                target_yaw = fmod(yaw_rad, M_PI_2);

                // Pixel σε μέτρα
                double x_m = (cx - cx_center) * Z_dist / fx;
                double y_m = (cy - cy_center) * Z_dist / fy;

                // Μετατροπή σε frame ρομπότ
                target_x = -y_m;
                target_y = -x_m + y_offset;

                // Σχεδίαση για επιβεβαίωση
                cv::rectangle(output_frame, bbox, cv::Scalar(0, 255, 0), 2);
                cv::putText(output_frame, color_name, cv::Point(bbox.x, bbox.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
            }
        }
    }

    // Service callback - επιστρέφει θέση και orientation για το ζητούμενο χρώμα
    void callbackPickTarget(const PickTarget::Request::SharedPtr request,
                        const PickTarget::Response::SharedPtr response)
    {
        if (request->target_color == "blue") {
            response->x = blue_x_;
            response->y = blue_y_;
            response->yaw = blue_yaw_;
            response->success = true;
        } 
        else if (request->target_color == "red") {
            response->x = red_x_;
            response->y = red_y_;
            response->yaw = red_yaw_;
            response->success = true;
        }
        else if (request->target_color == "green") {
        response->x = green_x_;
        response->y = green_y_;
        response->yaw = green_yaw_;
        response->success = true;
        }
        else {
            response->success = false;
            return;
        }

        // Σταθερές τιμές - pitch κάθετα, z ύψος, grasp_width πλάτος κύβου
        response->pitch = 3.14;
        response->z = 0.15;
        response->grasp_width = 0.025;
        
        RCLCPP_INFO(this->get_logger(), "Target [%s]: X=%.3f, Y=%.3f", 
                    request->target_color.c_str(), response->x, response->y);
    }

    // Θέσεις κύβων - ενημερώνονται κάθε frame
    double blue_x_, blue_y_, blue_yaw_;
    double red_x_, red_y_, red_yaw_;
    double green_x_, green_y_,green_yaw_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<Image>::SharedPtr image_sub_;
    rclcpp::Service<PickTarget>::SharedPtr optical_server_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PickTargetServerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}