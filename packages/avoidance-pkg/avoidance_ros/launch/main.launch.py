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
                'x_low': -0.5,
                'x_high': 0.5,
                'y_low': -0.05,
                'y_high': 0.06,
                'z_low': 0.0,
                'z_high': 1.0,
                'leaf_size': 0.01,
                'min_z': 100.0,
            }]
        )
    ])