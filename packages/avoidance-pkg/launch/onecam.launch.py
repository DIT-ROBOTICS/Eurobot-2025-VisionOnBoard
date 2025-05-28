import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('avoidance_ros')
    config_file = os.path.join(pkg_share, 'config', 'onecam_config.yaml')

    return LaunchDescription([
        Node(
            package='avoidance_ros',
            executable='onecam',
            name='onboard_filter',
            namespace='vision',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.065', '0.085', '0.305',
            '-0.1745', '0.5236', '-1.6581',
            'base_footprint', 'camera_link']
        )
    ])