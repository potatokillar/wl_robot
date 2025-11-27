import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # # Set the namespace for the node
        # PushRosNamespace('hdl_global_localization'),
        # Load parameters
        DeclareLaunchArgument(
            'general_config', 
            default_value=os.path.join(
                get_package_share_directory('hdl_global_localization'),
                'config',
                'general_config.yaml'
            )
        ),
        DeclareLaunchArgument(
            'bbs_config', 
            default_value=os.path.join(
                get_package_share_directory('hdl_global_localization'),
                'config',
                'bbs_config.yaml'
            )
        ),
        DeclareLaunchArgument(
            'fpfh_ransac_config', 
            default_value=os.path.join(
                get_package_share_directory('hdl_global_localization'),
                'config',
                'fpfh_ransac_config.yaml'
            )
        ),
        DeclareLaunchArgument(
            'fpfh_teaser_config', 
            default_value=os.path.join(
                get_package_share_directory('hdl_global_localization'),
                'config',
                'fpfh_teaser_config.yaml'
            )
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Declare the node
        Node(
            package='hdl_global_localization',
            executable='hdl_global_localization_node',
            output='screen',
            parameters=[
                LaunchConfiguration('general_config'),
                LaunchConfiguration('bbs_config'),
                LaunchConfiguration('fpfh_ransac_config'),
                LaunchConfiguration('fpfh_teaser_config'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
    ])