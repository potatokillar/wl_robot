from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    d435_rgb_node = Node(
        package="d435_rgb",
        executable='d435_rgb',
        parameters = [
            os.path.join(
                get_package_share_directory("d435_rgb"), 
                "config", 
                "d435_rgb.yaml"
            )
        ]
    )

    return LaunchDescription([
        d435_rgb_node
    ])