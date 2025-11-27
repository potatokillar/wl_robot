#############################################################################

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():

    # arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sensor'),
                'launch',
                'sensor.launch.py'
            )
        ),
        launch_arguments = {
            'sensor_type' : 'mid360',
            'mid360_custom_msg' : '1'
        }.items()
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            os.path.join(
                get_package_share_directory('navigation_bringup'),
                'config', 
                'fast_lio_mid360.yaml'
            ),
            {'use_sim_time': use_sim_time}],
        output='screen'
    )

    horizon_map_node = Node(
        package='horizon_map',
        executable='horizon_map_node',
        parameters=[
            os.path.join(
                get_package_share_directory('navigation_bringup'),
                'config',
                'horizon_map.yaml'),
            {'use_sim_time': use_sim_time}]
    )

    map_key_node = Node(
        package='navigation_key',
        executable='map_3d_key_node'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)

    ld.add_action(sensor_launch)
    ld.add_action(fast_lio_node)
    ld.add_action(horizon_map_node)
    ld.add_action(map_key_node)

    return ld