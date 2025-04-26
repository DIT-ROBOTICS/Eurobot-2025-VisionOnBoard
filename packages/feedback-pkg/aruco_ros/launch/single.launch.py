from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context, *args, **kwargs):

    aruco_single_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': 'd405_color_optical_frame',
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
        'detection_mode': LaunchConfiguration('detection_mode'),
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_single_params],
        remappings=[('/camera_info', '/realsense/d405/color/camera_info'),
                    ('/image', '/realsense/d405/color/image_rect_raw')],
    )

    return [aruco_single]


def generate_launch_description():

    # Declare namespace
    namespace = '/vision/aruco'

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='47',
        description='Marker ID. '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.03',
        description='Marker size in m. '
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Frame in which the marker pose will be referred. '
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='base_footprint',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement. ',
        choices=['SUBPIX', 'LINES', 'NONE'],
    )

    dectection_mode_arg = DeclareLaunchArgument(
        'detection_mode', default_value='DM_VIDEO_FAST',
        description='Detection Mode. ',
        choices=['DM_NORMAL', 'DM_VIDEO_FAST', 'DM_FAST'],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame)
    ld.add_action(corner_refinement_arg)
    ld.add_action(dectection_mode_arg)

    # Add namespace to the group
    namespace_group = GroupAction(
        actions=[
            OpaqueFunction(function=launch_setup)
        ],
        scoped=True
    )

    # Push the namespace
    ld.add_action(PushRosNamespace(namespace))
    ld.add_action(namespace_group)

    return ld