#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <memory>
#include "voxelgrid-cube.h"

class PointCloudFilter : public rclcpp::Node{
public:
    PointCloudFilter(const std::string& node_name,
                     const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node(node_name, options)
        {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        // Declare parameters
        declare_parameters();

        //Initialize comms
        setup_comms();
    }
private:
    void declare_parameters() {
        // Public parameters
        this->declare_parameter<double>("leaf_size", 0.05);
        this->declare_parameter<double>("min_z", 100);

        // Front camera parameters
        this->declare_parameter<double>("front_x_low", 0.16);
        this->declare_parameter<double>("front_x_high", 0.5);
        this->declare_parameter<double>("front_y_low", -0.5);
        this->declare_parameter<double>("front_y_high", 0.5);
        this->declare_parameter<double>("front_z_low", 0.015);
        this->declare_parameter<double>("front_z_high", 0.5);

        // Back camera parameters
        this->declare_parameter<double>("back_x_low", -0.5);
        this->declare_parameter<double>("back_x_high", -0.16);
        this->declare_parameter<double>("back_y_low", -0.5);
        this->declare_parameter<double>("back_y_high", 0.5);
        this->declare_parameter<double>("back_z_low", 0.015);
        this->declare_parameter<double>("back_z_high", 0.5);

        // Front camera
        front_cube_.x_low     = this->get_parameter("front_x_low").as_double();
        front_cube_.x_high    = this->get_parameter("front_x_high").as_double();
        front_cube_.y_low     = this->get_parameter("front_y_low").as_double();
        front_cube_.y_high    = this->get_parameter("front_y_high").as_double();
        front_cube_.z_low     = this->get_parameter("front_z_low").as_double();
        front_cube_.z_high    = this->get_parameter("front_z_high").as_double();

        // Back camera
        back_cube_.x_low     = this->get_parameter("back_x_low").as_double();
        back_cube_.x_high    = this->get_parameter("back_x_high").as_double();
        back_cube_.y_low     = this->get_parameter("back_y_low").as_double();
        back_cube_.y_high    = this->get_parameter("back_y_high").as_double();
        back_cube_.z_low     = this->get_parameter("back_z_low").as_double();
        back_cube_.z_high    = this->get_parameter("back_z_high").as_double();

        // Public parameters
        front_cube_.leaf_size = back_cube_.leaf_size = this->get_parameter("leaf_size").as_double();
        front_cube_.min_z     = back_cube_.min_z     = this->get_parameter("min_z").as_double();
    }
    void setup_comms() {
        // Subscribe and publish
        sub_frontcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/vision/onboard_front/depth/color/points", 10, std::bind(&PointCloudFilter::frontCloudCallback_, this, std::placeholders::_1));
        pub_frontcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vision/onboard_front/output", 10);

        sub_backcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/vision/onboard_back/depth/color/points", 10, std::bind(&PointCloudFilter::backCloudCallback_, this, std::placeholders::_1));
        pub_backcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vision/onboard_back/output", 10);

        // Stop robot publisher
        sub_frontstopRobot_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/vision/onboard_front/output", 10, std::bind(&PointCloudFilter::frontstopRobot_, this, std::placeholders::_1));
        sub_backstopRobot_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/vision/onboard_back/output", 10, std::bind(&PointCloudFilter::backstopRobot_, this, std::placeholders::_1));
        // pub_stopRobot_ = this->create_publisher<std_msgs::msg::Bool>("/stopRobot", rclcpp::QoS(1).reliable().transient_local());
        pub_stopRobot_ = this->create_publisher<std_msgs::msg::Bool>("/stopRobot", 10);

        // Topic below is just for test
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&PointCloudFilter::Direct, this, std::placeholders::_1));
    }

    void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
                      Cube& cube,
                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub) {
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
        cube.filterPointCloud(transformed_cloud, filtered_cloud);
        filtered_cloud.header = transformed_cloud.header;

        pub->publish(filtered_cloud);
    }

    void frontCloudCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        CameraSide();
        if (camera_side_ != "front") return;
        processCloud(cloud_msg, front_cube_, pub_frontcloud_);
    }
    
    void backCloudCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        CameraSide();
        if(camera_side_.empty()) {
            std::cout << "Velocity is empty, setting to stop." << std::endl;
            camera_side_ = "stop";
        }
        if (camera_side_ != "back") return;
        processCloud(cloud_msg, back_cube_, pub_backcloud_);
    }


    void frontstopRobot_(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        std_msgs::msg::Bool output;
        if (camera_side_ == "front") {
            output.data = front_cube_.stopRobot(cloud_msg);
            std::cout << camera_side_ << std::endl;
        } else {
            return;
        }
        pub_stopRobot_->publish(output);
    }
    void backstopRobot_(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        std_msgs::msg::Bool output;
        if (camera_side_ == "back") {
            output.data = front_cube_.stopRobot(cloud_msg);
            std::cout << camera_side_ << std::endl;
        } else {
            return;
        }
        pub_stopRobot_->publish(output);
    }

    void Direct(const geometry_msgs::msg::Twist::SharedPtr msg) {
        linear_x = msg->linear.x;
    }

    void CameraSide() {
        std_msgs::msg::Bool output;
        output.data = false;
        std::cout << "linear_x: " << linear_x << std::endl;
        if(linear_x > 0.0) {
            camera_side_ = "front";
        }
        if(linear_x < 0.0) {
            camera_side_ = "back";
        }
        if(linear_x == 0.0) {
            camera_side_ = "stop";
            pub_stopRobot_->publish(output);
        }
        if(camera_side_.empty()) {
            std::cout << "Velocity is empty, setting to stop." << std::endl;
            camera_side_ = "stop";
            pub_stopRobot_->publish(output);
        }
        std::cout << "camera_side_: " << camera_side_ << std::endl;
    }

    Cube front_cube_, back_cube_;
    std::string camera_side_ = "stop";
    double linear_x = 0.0;
    geometry_msgs::msg::Vector3 linear, angular;
    // TF2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_frontcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frontcloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_backcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_backcloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_frontstopRobot_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_backstopRobot_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stopRobot_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto filter = std::make_shared<PointCloudFilter>("onboard_filter", rclcpp::NodeOptions());

    rclcpp::spin(filter);
    rclcpp::shutdown();
    return 0;
}
