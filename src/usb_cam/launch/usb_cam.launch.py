from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='webcam_driver_node',
            name='camera',
            namespace='camera',  # <-- Adicione esta linha!
            output='screen',
            parameters=[{
                'video_device': '/dev/video2',
                'frame_width': 1920,
                'frame_height': 1080,
                'fps': 30
            }],
            remappings=[
                ('image_raw', 'image_raw')  # sem barra, fica /camera/image_raw
            ]
        )
    ])