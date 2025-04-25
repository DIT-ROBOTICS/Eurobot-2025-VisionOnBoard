#include <memory>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class DynamicTransformBroadcaster : public rclcpp::Node
{
public:
    DynamicTransformBroadcaster()
        : Node("dynamic_tf_broadcaster")
    {
        // Initialize the transform broadcaster
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create a timer to publish the transform periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // 33ms period (30Hz)
            std::bind(&DynamicTransformBroadcaster::broadcastTransform, this));
    }

private:
    void broadcastTransform()
    {
        // Create a transform message
        geometry_msgs::msg::TransformStamped transform;

        // Set the header
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "d405_depth_optical_frame";  // Parent frame
        transform.child_frame_id  = "d405_tf_frame";             // Child frame

        // Set the translation (position)
        transform.transform.translation.x = 0.0;  // X position
        transform.transform.translation.y = 0.0;  // Y position
        transform.transform.translation.z = 0.0;  // Z position

        // Set the rotation (orientation)
        tf2::Quaternion q;
        q.setRPY(0, -M_PI*8/9, 0);  // Roll, Pitch, Yaw (in radians)
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // Publish the transform
        broadcaster_->sendTransform(transform);

        RCLCPP_INFO(this->get_logger(), "Published dynamic transform from '%s' to '%s'",
                    transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTransformBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}