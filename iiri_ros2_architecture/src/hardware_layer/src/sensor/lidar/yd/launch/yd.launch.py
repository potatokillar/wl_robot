#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    params_declare = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('yd'), 
            'config', 
            'X2.yaml'
        ),
        description='FPath to the ROS2 config file to use.'
    )
    config_file = LaunchConfiguration('config_file')

    driver_node = LifecycleNode(
        package='yd_node',
        executable='yd_node',
        name='yd_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
    )
    
    return LaunchDescription([
        params_declare,
        driver_node,
    ])
