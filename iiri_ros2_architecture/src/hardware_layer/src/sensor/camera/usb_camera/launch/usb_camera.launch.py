from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    usb_camera_node = Node(
        package="usb_camera",
        executable='usb_camera',
        parameters = [
            os.path.join(
                get_package_share_directory("usb_camera"), 
                "config", 
                "usb_camera.yaml"
            )
        ]
    )

    return LaunchDescription([
        usb_camera_node
    ])