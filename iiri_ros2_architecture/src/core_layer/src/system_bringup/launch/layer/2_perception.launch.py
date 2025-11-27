#!/usr/bin/env python3
"""
Perception Layer Launch File
启动感知层所有节点：相机、扬声器、语音识别、TTS等
依赖：hardware_layer (1_hardware.launch.py)
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
    """生成感知层启动描述"""

    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera PTZ'
    )

    enable_audio_arg = DeclareLaunchArgument(
        'enable_audio',
        default_value='true',
        description='Enable audio (speaker, TTS, speech recognition)'
    )

    enable_speech_recognition_arg = DeclareLaunchArgument(
        'enable_speech_recognition',
        default_value='true',
        description='Enable speech recognition (requires libalibabacloud-idst-speech.so)'
    )

    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_audio = LaunchConfiguration('enable_audio')
    enable_speech_recognition = LaunchConfiguration('enable_speech_recognition')

    # 包含硬件层启动文件
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
        }.items()
    )

    # 相机PTZ节点（条件启动）
    camera_config = None
    try:
        camera_config = os.path.join(
            get_package_share_directory('camera_ptz'),
            'config', 'orin.yaml'
        )
    except:
        pass

    camera_params = [camera_config] if camera_config and os.path.exists(camera_config) else []
    camera_ptz_node = Node(
        package='camera_ptz',
        executable='camera_ptz_node',
        name='camera_ptz',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        parameters=camera_params if camera_params else [{
            'use_sim_time': use_sim_time,
        }]
    )

    # 扬声器节点
    speaker_config = None
    try:
        speaker_config = os.path.join(
            get_package_share_directory('speaker'),
            'config', 'speaker.yaml'
        )
    except:
        pass

    speaker_params = [speaker_config] if speaker_config and os.path.exists(speaker_config) else []
    speaker_node = Node(
        package='speaker',
        executable='speaker_node',
        name='speaker',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        condition=IfCondition(LaunchConfiguration('enable_audio')),
        parameters=speaker_params
    )

    # TTS节点
    tts_node = Node(
        package='tts',
        executable='tts_node',
        name='tts',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        condition=IfCondition(LaunchConfiguration('enable_audio'))
    )

    # 语音识别节点
    asr_config = None
    try:
        asr_config = os.path.join(
            get_package_share_directory('speech_recognition'),
            'config', 'speech_recognition_node.yaml'
        )
    except:
        pass

    asr_params = [asr_config] if asr_config and os.path.exists(asr_config) else []
    speech_recognition_node = Node(
        package='speech_recognition',
        executable='speech_recognition_node',
        name='speech_recognition',
        output='own_log',
        arguments=['--ros-args', '--disable-external-lib-logs'],
        condition=IfCondition(LaunchConfiguration('enable_speech_recognition')),
        parameters=asr_params
    )

    # 日志信息
    log_info = LogInfo(msg='[system_bringup] Perception Layer started')

    return LaunchDescription([
        use_sim_time_arg,
        enable_camera_arg,
        enable_audio_arg,
        enable_speech_recognition_arg,
        hardware_launch,
        log_info,
        camera_ptz_node,
        speaker_node,
        tts_node,
        speech_recognition_node,
    ])
