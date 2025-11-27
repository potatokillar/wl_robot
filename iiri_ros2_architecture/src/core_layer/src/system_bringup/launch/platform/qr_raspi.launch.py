#!/usr/bin/env python3
"""
QR Raspberry Pi Platform Launch File
适用于树莓派平台的QR机器人启动文件
使用 intelligence_layer (3层) 以降低计算负载
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成树莓派平台启动描述"""

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

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera PTZ'
    )

    enable_audio_arg = DeclareLaunchArgument(
        'enable_audio',
        default_value='true',
        description='Enable audio modules'
    )

    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_audio = LaunchConfiguration('enable_audio')

    # 包含智能层启动文件（3层架构，不包含应用层）
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
            'enable_navigation': enable_navigation,
            'enable_camera': enable_camera,
            'enable_audio': enable_audio,
        }.items()
    )

    # 平台启动日志
    log_info = LogInfo(
        msg='[system_bringup] QR Raspberry Pi Platform started (3-layer architecture)'
    )

    return LaunchDescription([
        use_sim_time_arg,
        enable_navigation_arg,
        enable_camera_arg,
        enable_audio_arg,
        log_info,
        intelligence_launch,
    ])
