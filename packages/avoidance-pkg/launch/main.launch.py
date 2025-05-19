import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('avoidance_ros')
    config_file = os.path.join(pkg_share, 'launch', 'avoidance_config.yml')

    return LaunchDescription([
        Node(
            package='avoidance_ros',
            executable='main',
            name='onboard_filter',
            namespace='vision',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='avoidance_ros',
            executable='test_vel',
            name='velocity_publisher',
            namespace='vision',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            name='frontcam_transform_publisher',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '0.065', '0.085', '0.305',
            '-0.1745', '0.5236', '-1.6581',
            'base_footprint',
            'camera1_link'],
        ),
        Node(
            name='backcam_transform_publisher',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '-0.065', '0.085', '0.305',
            '3.4034', '0.5236', '-1.3963',
            'base_footprint',
            'camera2_link'],
        )
    ])