from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def generate_launch_description():
    
    sensor_type = LaunchConfiguration('sensor_type')   # 外层launch通过，sensor_type指定传感器类型
    mid360_custom_msg = LaunchConfiguration('mid360_custom_msg')   # 外层launch通过，sensor_type指定传感器类型

    # --- 参数 --- #
    declare_sensor_type = DeclareLaunchArgument(
        "sensor_type",
        default_value = 'mid360',
        description   = 'type of sensor, "mid360、rs16、yd、d435_rgb、usb_camera"')

    declare_mid360_custom_msg = DeclareLaunchArgument(
        "mid360_custom_msg",
        default_value = '0',
        description   = '0: use the default pointcloud2 msg for localization, 1: use the custom_msg for fast_lio to mapping')

    # --- 启动传感器 --- #
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([             # 相比os.path.join可以传入LaunchConfiguration类型的运行时解析参数
                FindPackageShare(sensor_type), # 相比get_package_share_directory()可以传入LaunchConfiguration类型的运行时解析参数
                'launch',
                PythonExpression(['"',sensor_type,'" + ".launch.py"', ]) # 使用python表达式将运行时解析的参数值转化为字符串进行拼接,灵活地获取launch文件名
            ])
        ),
        launch_arguments={
            'mid360_custom_msg' : mid360_custom_msg
        }.items()
    )    

    ld = LaunchDescription()
    
    ld.add_action(declare_sensor_type)
    ld.add_action(declare_mid360_custom_msg)

    ld.add_action(sensor_launch)

    return ld
