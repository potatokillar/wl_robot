#!/usr/bin/env python3
"""
Application Layer Launch File
启动应用层所有节点：开发服务器、键盘控制、远程控制、录音等
依赖：intelligence_layer (3_intelligence.launch.py)
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
    """生成应用层启动描述"""

    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_dev_server_arg = DeclareLaunchArgument(
        'enable_dev_server',
        default_value='true',
        description='Enable development server'
    )

    enable_remote_ctrl_arg = DeclareLaunchArgument(
        'enable_remote_ctrl',
        default_value='true',
        description='Enable remote control'
    )

    enable_key_control_arg = DeclareLaunchArgument(
        'enable_key_control',
        default_value='false',
        description='Enable keyboard control (debugging only)'
    )

    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_dev_server = LaunchConfiguration('enable_dev_server')
    enable_remote_ctrl = LaunchConfiguration('enable_remote_ctrl')
    enable_key_control = LaunchConfiguration('enable_key_control')

    # 包含智能层启动文件
    intelligence_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('system_bringup'),
                'launch',
                'layer',
                '3_intelligence.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 开发服务器节点（条件启动）
    dev_server_node = Node(
        package='dev_server',
        executable='dev_server_node',
        name='dev_server',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        condition=IfCondition(LaunchConfiguration('enable_dev_server')),
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # 远程控制节点（条件启动）
    remote_config = None
    try:
        remote_config = os.path.join(
            get_package_share_directory('remote_ctrl'),
            'config', 'orin.yaml'
        )
    except:
        pass

    remote_params = [remote_config] if remote_config and os.path.exists(remote_config) else []
    remote_ctrl_node = Node(
        package='remote_ctrl',
        executable='remote_ctrl_node',
        name='remote_ctrl',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        condition=IfCondition(LaunchConfiguration('enable_remote_ctrl')),
        parameters=remote_params if remote_params else [{
            'use_sim_time': use_sim_time,
        }]
    )

    # 录音节点（麦克风收音）
    record_config = None
    try:
        record_config = os.path.join(
            get_package_share_directory('record'),
            'config', 'record.yaml'
        )
    except:
        pass

    record_params = [record_config] if record_config and os.path.exists(record_config) else []
    record_node = Node(
        package='record',
        executable='record_node',
        name='record_node',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        parameters=record_params
    )

    # 键盘控制节点（条件启动，仅用于调试）
    key_control_node = Node(
        package='key_control',
        executable='key_control_node',
        name='key_control',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        condition=IfCondition(LaunchConfiguration('enable_key_control')),
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # 日志信息
    log_info = LogInfo(msg='[system_bringup] Application Layer started')

    return LaunchDescription([
        use_sim_time_arg,
        enable_dev_server_arg,
        enable_remote_ctrl_arg,
        enable_key_control_arg,
        intelligence_launch,
        log_info,
        dev_server_node,
        remote_ctrl_node,
        record_node,
        key_control_node,
    ])
