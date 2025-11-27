#!/usr/bin/env python3
"""
QR Debug Platform Launch File
用于调试的最小化启动配置
只启动硬件层（1层），方便快速测试和调试
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成调试平台启动描述"""

    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='qr',
        description='Robot model for debugging'
    )

    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')

    # 只包含硬件层启动文件（最小化配置）
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('system_bringup'),
                'launch',
                'layer',
                '1_hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
        }.items()
    )

    # 平台启动日志
    log_info = LogInfo(
        msg='[system_bringup] QR Debug Platform started (1-layer minimal configuration)'
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_model_arg,
        log_info,
        hardware_launch,
    ])
