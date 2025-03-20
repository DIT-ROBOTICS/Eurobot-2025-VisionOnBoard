#include <memory>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class TransformBroadcaster : public rclcpp::Node
{
public:
    TransformBroadcaster()
        : Node("tf_static_broadcaster")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Set Static Transform
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "map";         // Parent
        static_transform.child_frame_id = "base_link";    // Child
        static_transform.transform.translation.x = 1.0;   // X Move
        static_transform.transform.translation.y = 0.0;   // Y Move
        static_transform.transform.translation.z = 0.5;   // Z Move

        // Set Rotation
        tf2::Quaternion q;
        q.setRPY(M_PI/2, 0, 0);
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        // Publish Static Transform
        broadcaster_->sendTransform(static_transform);

        RCLCPP_INFO(this->get_logger(), "Published static transform from 'map' to 'base_link'");
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
