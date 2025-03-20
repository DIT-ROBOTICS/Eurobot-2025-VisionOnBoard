#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>

#include "voxelgrid-cube.h"

class PointCloudFilter : public rclcpp::Node{
public:
    Cube cube_instance;
    double leaf_size = 0.05;

    PointCloudFilter()
        : Node("point_cloud_filter") {
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
        cube_instance.x_low = this->get_parameter("x_low").as_double();
        cube_instance.x_high = this->get_parameter("x_high").as_double();
        cube_instance.y_low = this->get_parameter("y_low").as_double();
        cube_instance.y_high = this->get_parameter("y_high").as_double();
        cube_instance.z_low = this->get_parameter("z_low").as_double();
        cube_instance.z_high = this->get_parameter("z_high").as_double();
        cube_instance.leaf_size = this->get_parameter("leaf_size").as_double();
        cube_instance.min_z = this->get_parameter("min_z").as_double();
        
        // Subscribe and publish
        sub_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/realsense/d405/depth/color/points", 10, std::bind(&PointCloudFilter::cloudCallback, this, std::placeholders::_1));
        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/realsense/d405/output", 10);

        sub_bool_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/realsense/d405/transformed_output", 10, std::bind(&PointCloudFilter::StopRobot, this, std::placeholders::_1));
        pub_bool_ = this->create_publisher<std_msgs::msg::Bool>("/stopRobot", 10);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        sensor_msgs::msg::PointCloud2 output;
        cube_instance.filterPointCloud(cloud_msg, output);
        output.header = cloud_msg->header;
        pub_pointcloud_->publish(output);
    }

    void StopRobot(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        std_msgs::msg::Bool output;
        output.data = cube_instance.StopRobot(cloud_msg);
        pub_bool_->publish(output);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_bool_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bool_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto filter = std::make_shared<PointCloudFilter>();

    rclcpp::spin(filter);
    rclcpp::shutdown();
    return 0;
}