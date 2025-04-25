#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer"),
                              tf_buffer_(this->get_clock()),
                              tf_listener_(tf_buffer_)
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/realsense/d405/depth/color/points", 10,
            std::bind(&PointCloudTransformer::cloud_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/realsense/d405/transformed_output", 10);
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::string target_frame = "d405_tf_frame"; // Transform to desired target frame
        try
        {
            // Get the transformation
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform(
                    target_frame,
                    msg->header.frame_id,
                    msg->header.stamp,
                    rclcpp::Duration::from_seconds(1.0)
                );

            // Transform the point cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);

            // Publish the transformed point cloud
            transformed_cloud.header.frame_id = target_frame;
            pub_->publish(transformed_cloud);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
