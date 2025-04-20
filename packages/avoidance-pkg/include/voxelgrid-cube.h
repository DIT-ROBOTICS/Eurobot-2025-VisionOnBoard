#ifndef _CUBE_H_
#define _CUBE_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class Cube {
public:
    double x_low = -0.5, x_high = 0.5, y_low = -0.5, y_high = 0.5, z_low = 0, z_high = 1;
    double leaf_size = 0.05;
    double min_z = 100;

    void filterPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input, sensor_msgs::msg::PointCloud2& output);
    bool StopRobot(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);
};

#endif // _CUBE_H_
