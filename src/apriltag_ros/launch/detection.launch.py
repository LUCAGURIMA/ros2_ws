from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_detector',
            name='apriltag_detector',
            parameters=[{'tag_size': 0.16}],
            remappings=[
                ('/image_raw', '/usb_cam/image_raw'),
                ('/camera_info', '/usb_cam/camera_info')
            ]
        )
    ])