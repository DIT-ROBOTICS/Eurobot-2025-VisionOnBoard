import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('avoidance_ros')
    config_file = os.path.join(pkg_share, 'config', 'avoidance_config.yml')

    return LaunchDescription([
        Node(
            package='avoidance_ros',
            executable='test_vel',
            name='velocity_publisher',
            namespace='vision',
            output='screen',
            parameters=[config_file],
        ),

    ])