import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('avoidance_ros')
    config_file = os.path.join(pkg_share, 'config', 'avoidance_config.yaml')

    return LaunchDescription([
        Node(
            package='avoidance_ros',
            executable='main',
            name='onboard_filter',
            namespace='vision',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0905', '0.0', '0.303', '0', '0.3496', '-1.5707963268',
                      'base_footprint', 'camera_link']
        )
    ])