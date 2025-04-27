from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='rover_camera',
            remappings=[
                ('/image_raw', '/rover_camera/image_raw'),
            ],
            parameters=[{
                'image_topic': '/rover_camera/image_raw'
            }]
        )
    ])
