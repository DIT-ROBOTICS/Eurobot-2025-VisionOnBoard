#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

class VelPub : public rclcpp::Node {
public:
    VelPub(const std::string& node_name,
           const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node(node_name, options)
    {
        setup_comms();
    }

private: 
    void setup_comms() {
        // Topic below is to determine the dierection of the robot
        sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/vision/onboard/vel", 10, std::bind(&VelPub::camera_side_, this));
        pub_vel_ = this->create_publisher<std_msgs::msg::String>("/vision/onboard/vel", 10);
    }

    void camera_side_(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std_msgs::msg::string msg;
        if(msg->linear.x >= 0.0) {
            msg.data = "front";
        }
        if(msg->linear.x < 0.0) {
            msg.data = "back";
        }
        
        pub_vel_->publish(msg);
    }
    // Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_vel_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelPub>("side_publisher");
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}