#include "voxelgrid-cube.h"

extern double x_low, x_high, y_low, y_high, z_low, z_high, leaf_size, min_z;

void Cube::filterPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg, sensor_msgs::msg::PointCloud2& output) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Filter X
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_low, x_high);
    pass.filter(*cloud_filtered);

    // Filter Y
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_low, y_high);
    pass.filter(*cloud_filtered);

    // Filter Z
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_low, z_high);
    pass.filter(*cloud_filtered);

    // Voxel Grid
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_filtered);

    pcl::toROSMsg(*cloud_filtered, output);
}

bool Cube::StopRobot(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    for(const auto& point : cloud->points) {
        if(point.z < min_z) {
            return true;
        }
    }
    return false;
}