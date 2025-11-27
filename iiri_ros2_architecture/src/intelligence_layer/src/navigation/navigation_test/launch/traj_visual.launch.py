#############################################################################

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
import os

from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():

    traj_visual_test_node = Node(
        package="navigation_test",
        executable="traj_visual_node",
        parameters=[
            os.path.join(get_package_share_directory("navigation_test"), 'config', 'traj_visual_node.yaml'), 
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('navigation_test'),
                'rviz',
                'traj_visual.rviz'
            )
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(traj_visual_test_node)
    # ld.add_action(lookup_transform_test_node)
    ld.add_action(rviz_node)

    return ld