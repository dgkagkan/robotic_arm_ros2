#include "rclcpp/rclcpp.hpp"
#include "robotic_arm_interfaces/srv/pick_target.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"
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
            "/camera/image",
            10,
            std::bind(&PickTargetServerNode::camera_callback, this, _1),
            sub_options
        );

        depth_sub_ = this->create_subscription<Image>(
            "/camera/depth_image", 10,
            std::bind(&PickTargetServerNode::depth_callback, this, _1),
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
            process_color(blue_mask, frame, blue_x_, blue_y_, blue_z_, blue_yaw_, "blue");

            // Κόκκινη μάσκα (δύο εύρη γιατί το κόκκινο τυλίγεται στο HSV)
            cv::Mat mask1, mask2, red_mask;
            cv::inRange(hsv_frame, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(hsv_frame, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);
            cv::addWeighted(mask1, 1.0, mask2, 1.0, 0.0, red_mask);
            process_color(red_mask, frame, red_x_, red_y_, red_z_, red_yaw_, "red");

            // Πράσινη μάσκα
            cv::Mat green_mask;
            cv::inRange(hsv_frame, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), green_mask);
            process_color(green_mask, frame, green_x_, green_y_, green_z_, green_yaw_, "green");

            // Δημοσίευση debug εικόνας
            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            debug_pub_->publish(*debug_msg);

            
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        }
    }

    // Μετατροπή pixel σε μέτρα (pinhole camera model) και εύρεση yaw
    void process_color(cv::Mat mask, cv::Mat &output_frame, double &target_x, double &target_y,double &target_height, double &target_yaw, std::string color_name) 
    {
        // Camera intrinsics
        const double fx = 381.36116;
        const double fy = 381.36114;
        const double cx_center = 320.0;
        const double cy_center = 240.0;
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
                yaw_rad = fmod(yaw_rad + M_PI, 2 * M_PI) - M_PI;
                target_yaw = yaw_rad;

                if (!depth_image_.empty() &&
                    cy < depth_image_.rows && cx < depth_image_.cols)
                {
                    // Πάρε 5x5 περιοχή γύρω από το κέντρο
                    std::vector<float> valid_depths;
                    int window = 5;
                    for (int dy = -window; dy <= window; dy++) {
                        for (int dx = -window; dx <= window; dx++) {
                            int nx = cx + dx;
                            int ny = cy + dy;
                            if (nx >= 0 && nx < depth_image_.cols &&
                                ny >= 0 && ny < depth_image_.rows) {
                                float d = depth_image_.at<float>(ny, nx);
                                if (std::isfinite(d) && d > 0.0f)
                                    valid_depths.push_back(d);
                            }
                        }
                    }

                    if (!valid_depths.empty()) {
                        float sum = 0;
                        for (auto d : valid_depths) sum += d;
                        target_height = 2.135 - (double)(sum / valid_depths.size());
                        target_dist_ = (double)(sum / valid_depths.size());
                        RCLCPP_INFO(this->get_logger(), "[%s] Depth: %.3f m", color_name.c_str(), target_height);
                    } else {
                        target_height = 0.15;
                    }
                }

                const double Z_dist = target_dist_;

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
            response->z = blue_z_;
            response->yaw = blue_yaw_;
            response->success = true;
        } 
        else if (request->target_color == "red") {
            response->x = red_x_;
            response->y = red_y_;
            response->z = red_z_;
            response->yaw = red_yaw_;
            response->success = true;
        }
        else if (request->target_color == "green") {
        response->x = green_x_;
        response->y = green_y_;
        response->z = green_z_;
        response->yaw = green_yaw_;
        response->success = true;
        }
        else {
            response->success = false;
            return;
        }

        // Σταθερές τιμές - pitch κάθετα, z ύψος, grasp_width πλάτος κύβου
        response->pitch = 3.14;
        response->grasp_width = 0.025;
        
        RCLCPP_INFO(this->get_logger(), "Target [%s]: X=%.3f, Y=%.3f", 
                    request->target_color.c_str(), response->x, response->y);
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Depth CV Bridge Error: %s", e.what());
        }
    }

    // Θέσεις κύβων - ενημερώνονται κάθε frame
    double target_dist_;
    double blue_x_, blue_y_, blue_yaw_, blue_z_ = 0.15;
    double red_x_, red_y_, red_yaw_, red_z_ = 0.15;
    double green_x_, green_y_, green_yaw_, green_z_ = 0.15;
    cv::Mat depth_image_;
    rclcpp::Subscription<Image>::SharedPtr depth_sub_;
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