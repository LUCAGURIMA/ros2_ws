from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_cam',
                plugin='WebcamDriver',
                name='usb_cam',
                namespace='camera',
                parameters=[{
                    'video_device': '/dev/video2',
                    'frame_width': 1920,
                    'frame_height': 1080,
                    'fps': 30
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify',
                namespace='camera',
                remappings=[
                    ('image', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='apriltag',
                remappings=[
                    ('/apriltag/image_rect', '/camera/image_rect'),
                    ('/camera/camera_info', '/camera/camera_info')
                ],
                parameters=[{
                    'camera_info_url': '',
                    'from': '$(find-pkg-share apriltag_ros)/cfg/tags_36h11.yaml'
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])