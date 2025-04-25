from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avoidance_ros',
            executable='main',
            name='point_cloud_filter',
            output='screen',
            parameters=[{
                'x_low'    : -0.0,
                'x_high'   : 0.26,
                'y_low'    : -0.5,
                'y_high'   : 0.5,
                'z_low'    : -0.5,
                'z_high'   : 0.5,
                'leaf_size': 0.01,
                'min_z'    : 0.2,
            }]
        ),

        Node(
            package='avoidance_ros',
            executable='transform',
            name='pointcloud_transformer',
            output='screen'
        ),
        Node(
            package='avoidance_ros',
            executable='tf_broadcaster',
            name='dynamic_tf_broadcaster',
            output='screen'
        )
    ])