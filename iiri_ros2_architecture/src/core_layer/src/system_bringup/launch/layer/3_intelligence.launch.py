#!/usr/bin/env python3
"""
Intelligence Layer Launch File
启动智能层所有节点：导航、行为树管理、智能跟随、路径跟踪等
依赖：perception_layer (2_perception.launch.py)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成智能层启动描述"""

    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable navigation module'
    )

    enable_follow_arg = DeclareLaunchArgument(
        'enable_follow',
        default_value='false',
        description='Enable smart follow module'
    )

    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_follow = LaunchConfiguration('enable_follow')

    # 包含感知层启动文件
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('system_bringup'),
                'launch',
                'layer',
                '2_perception.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 行为树管理节点
    bt_config = None
    try:
        bt_config = os.path.join(
            get_package_share_directory('bt_manager'),
            'config', 'qr.yaml'
        )
    except:
        pass

    bt_params = [bt_config] if bt_config and os.path.exists(bt_config) else []
    bt_manager_node = Node(
        package='bt_manager',
        executable='bt_manager_node',
        name='bt_manager',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        parameters=bt_params if bt_params else [{
            'use_sim_time': use_sim_time,
        }]
    )

    # 小智聊天节点
    xiaozhi_config = None
    try:
        xiaozhi_config = os.path.join(
            get_package_share_directory('xiaozhi'),
            'config', 'xiaozhi_node.yaml'
        )
    except:
        pass

    xiaozhi_params = [xiaozhi_config] if xiaozhi_config and os.path.exists(xiaozhi_config) else []
    xiaozhi_node = Node(
        package='xiaozhi',
        executable='xiaozhi_node',
        name='xiaozhi',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        parameters=xiaozhi_params
    )

    # 注意: path_tracker 是一个库包，不是独立节点
    # 它由 navigation_core 等其他包使用

    # 智能跟随节点（条件启动）
    smart_follow_node = Node(
        package='smart_follow',
        executable='smart_follow_node',
        name='smart_follow',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        condition=IfCondition(LaunchConfiguration('enable_follow')),
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # 日志信息
    log_info = LogInfo(msg='[system_bringup] Intelligence Layer started')

    return LaunchDescription([
        use_sim_time_arg,
        enable_navigation_arg,
        enable_follow_arg,
        perception_launch,
        log_info,
        bt_manager_node,
        xiaozhi_node,
        smart_follow_node,
    ])
