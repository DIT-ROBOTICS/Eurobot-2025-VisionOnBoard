#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <memory>

#include "voxelgrid-cube.h"

class PointCloudFilter : public rclcpp::Node{
public:
    Cube cube_instance;
    double leaf_size = 0.05;

    PointCloudFilter(const std::string& node_name)
        : Node(node_name) {
        // Initialize the tf2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Declare parameters
        this->declare_parameter<double>("x_low", -0.5);
        this->declare_parameter<double>("x_high", 0.5);
        this->declare_parameter<double>("y_low", -0.5);
        this->declare_parameter<double>("y_high", 0.5);
        this->declare_parameter<double>("z_low", 0);
        this->declare_parameter<double>("z_high", 1);
        this->declare_parameter<double>("leaf_size", 0.05);
        this->declare_parameter<double>("min_z", 100);

        // Get parameters
        cube_instance.x_low     = this->get_parameter("x_low").as_double();
        cube_instance.x_high    = this->get_parameter("x_high").as_double();
        cube_instance.y_low     = this->get_parameter("y_low").as_double();
        cube_instance.y_high    = this->get_parameter("y_high").as_double();
        cube_instance.z_low     = this->get_parameter("z_low").as_double();
        cube_instance.z_high    = this->get_parameter("z_high").as_double();
        cube_instance.leaf_size = this->get_parameter("leaf_size").as_double();
        cube_instance.min_z     = this->get_parameter("min_z").as_double();
        
        // Subscribe and publish
        sub_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/vision/onboard/depth/color/points", 10, std::bind(&PointCloudFilter::cloudCallback, this, std::placeholders::_1));
        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vision/onboard/output", 10);

        sub_bool_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/vision/onboard/output", 10, std::bind(&PointCloudFilter::stopRobot, this, std::placeholders::_1));
        pub_bool_ = this->create_publisher<std_msgs::msg::Bool>("/stopRobot", 10);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // Transform the point cloud to the base_footprint frame
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "base_footprint",
                cloud_msg->header.frame_id,
                tf2::TimePointZero
            );
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(*cloud_msg, transformed_cloud, transform);
        transformed_cloud.header.frame_id = "base_footprint";

        sensor_msgs::msg::PointCloud2 filtered_cloud;
        cube_instance.filterPointCloud(transformed_cloud, filtered_cloud);
        filtered_cloud.header = transformed_cloud.header;
        pub_pointcloud_->publish(filtered_cloud);
    }

    void stopRobot(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        std_msgs::msg::Bool output;
        output.data = cube_instance.stopRobot(cloud_msg);
        pub_bool_->publish(output);
    }

private:
    // TF2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_bool_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bool_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("temp_parameter_reader");
    std::string node_name = node->declare_parameter<std::string>("name", "onboard_filter");
    node.reset(); 

    auto filter = std::make_shared<PointCloudFilter>(node_name);

    rclcpp::spin(filter);
    rclcpp::shutdown();
    return 0;
}