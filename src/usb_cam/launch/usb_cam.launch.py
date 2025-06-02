from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='webcam_driver',
            name='webcam_driver',
            output='screen',
            parameters=[{
                # Parâmetros opcionais podem ser adicionados aqui
            }]
        )
    ])