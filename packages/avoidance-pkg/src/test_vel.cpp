#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

class VelPub : public rclcpp::Node {
public:
    VelPub(const std::string& node_name,
           const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node(node_name, options)
    {
        // Declare parameters
        declare_parameters();

        // Initialize comms
        setup_comms();
    }
    // Initialize velocity message
    void publish_velocity() {
        geometry_msgs::msg::Twist msg;
        
        // Get current parameter values
        this->get_parameter("linear.x", msg.linear.x);
        this->get_parameter("linear.y", msg.linear.y);
        this->get_parameter("linear.z", msg.linear.z);
        this->get_parameter("angular.x", msg.angular.x);
        this->get_parameter("angular.y", msg.angular.y);
        this->get_parameter("angular.z", msg.angular.z);
        
        pub_vel_->publish(msg);
    }

private:
    void declare_parameters() {
        // Linear and Angle parameters
        this->declare_parameter<double>("linear.x", 0.0);
        this->declare_parameter<double>("linear.y", 0.0);
        this->declare_parameter<double>("linear.z", 0.0);
        this->declare_parameter<double>("angular.x", 0.0);
        this->declare_parameter<double>("angular.y", 0.0);
        this->declare_parameter<double>("angular.z", 0.0);
    }

    void setup_comms() {
        // Topic below is just for test
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/vision/onboard/vel", 10);
    }
    // Subscribers and publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelPub>("velocity_publisher");
    
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        node->publish_velocity();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}