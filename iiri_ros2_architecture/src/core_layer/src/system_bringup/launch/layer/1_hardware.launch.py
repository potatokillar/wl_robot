#!/usr/bin/env python3
"""
Hardware Layer Launch File
启动硬件层所有必需的节点：运动控制、机器人底盘等
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成硬件层启动描述"""

    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='qr',
        description='Robot model (qr, iiri, etc.)'
    )

    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation mode (skip config files if missing)'
    )

    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')

    # 配置文件路径（使用try-except处理包不存在的情况）
    motion_config = None
    base_config = None

    try:
        motion_config = os.path.join(
            get_package_share_directory('motion_control'),
            'config', 'qr_local.yaml'
        )
    except:
        pass

    try:
        base_config = os.path.join(
            get_package_share_directory('robot_base'),
            'config', 'default.yaml'
        )
    except:
        pass

    # 运动控制节点 (motion_control_node)
    motion_params = [motion_config] if motion_config and os.path.exists(motion_config) else []
    motion_control_node = Node(
        package='motion_control',
        executable='motion_control_node',
        name='motion_control',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        parameters=motion_params if motion_params else [{
            'use_sim_time': use_sim_time,
        }]
    )

    # 机器人底盘节点
    base_params = [base_config] if base_config and os.path.exists(base_config) else []
    robot_base_node = Node(
        package='robot_base',
        executable='robot_base_node',
        name='robot_base',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        parameters=base_params if base_params else [{
            'use_sim_time': use_sim_time,
        }]
    )

    # 日志信息
    log_info = LogInfo(msg='[system_bringup] Hardware Layer started')

    return LaunchDescription([
        use_sim_time_arg,
        robot_model_arg,
        simulation_mode_arg,
        log_info,
        motion_control_node,
        robot_base_node,
    ])
