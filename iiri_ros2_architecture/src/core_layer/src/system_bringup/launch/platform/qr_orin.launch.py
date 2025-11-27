#!/usr/bin/env python3
"""
QR NVIDIA Orin Platform Launch File
适用于 NVIDIA Orin 平台的QR机器人启动文件
使用完整的 application_layer (4层) 以充分利用强大的计算能力
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成 Orin 平台启动描述"""

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
    enable_dev_server = LaunchConfiguration('enable_dev_server')
    enable_remote_ctrl = LaunchConfiguration('enable_remote_ctrl')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_audio = LaunchConfiguration('enable_audio')

    # 包含完整应用层启动文件（4层架构）
    application_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('system_bringup'),
                'launch',
                'layer',
                '4_application.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_dev_server': enable_dev_server,
            'enable_remote_ctrl': enable_remote_ctrl,
            'enable_navigation': enable_navigation,
            'enable_camera': enable_camera,
            'enable_audio': enable_audio,
        }.items()
    )

    # 平台启动日志
    log_info = LogInfo(
        msg='[system_bringup] QR NVIDIA Orin Platform started (4-layer architecture)'
    )

    return LaunchDescription([
        use_sim_time_arg,
        enable_dev_server_arg,
        enable_remote_ctrl_arg,
        enable_navigation_arg,
        enable_camera_arg,
        enable_audio_arg,
        log_info,
        application_launch,
    ])
