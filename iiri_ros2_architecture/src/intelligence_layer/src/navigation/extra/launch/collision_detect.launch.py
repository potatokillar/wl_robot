from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description(): 

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sensor'),
                'launch',
                'sensor.launch.py'
            )
        ),
        launch_arguments = {
            'sensor_type' : 'rs16'
        }.items()
    )
    
    collision_detect_node = Node(
        package = 'extra',
        executable = 'collision_detect',
        parameters = [{
            'horizon_thresh_dist' : 10.0,
            'horizon_thresh_ang' : 2.0
        }],
        remappings = [
            ('point_cloud_in', '/rslidar_points')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['--display-config', 
            os.path.join(
                get_package_share_directory('extra'),
                'rviz',
                'collision_detect.rviz'
            )
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(sensor_launch)
    ld.add_action(collision_detect_node)
    ld.add_action(rviz_node)

    return ld