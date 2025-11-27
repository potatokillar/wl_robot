import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import psutil
import socket, json
import netifaces

# 长IP网卡优先级列表
NET_CARD_LIST = ("enP8p1s0", "eno1", "wlan0", "enp5s0")

################### user configure parameters for ros2 start ###################
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'laser_link'
imu_frame_id  = 'imu_link'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"imu_frame_id": imu_frame_id},
    # {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def save_target_ip():

    # 获取所有网卡信息
    interface = psutil.net_if_addrs()

    # 按优先级匹配网卡
    for net_card in NET_CARD_LIST:
        if net_card in interface:
            for addr in interface[net_card]:
                if addr.family == socket.AF_INET:  # IPv4地址
                    ip_parts = addr.address.split('.')
                    if len(ip_parts) == 4: # 确保IP地址格式正确
                        return f"{'.'.join(ip_parts[:4])}"
    raise Exception("未匹配到网卡")
    
def update_mid360_config(file_path, new_ip):
    """更新MID360配置文件"""
    try:
        with open(file_path, 'r') as f:
            config = json.load(f)
        
        if 'MID360' in config and 'host_net_info' in config['MID360']:
            host_net_info = config['MID360']['host_net_info']
            for key in ['cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip']:
                host_net_info[key] = new_ip
            if host_net_info.get('log_data_ip', '').strip():
                host_net_info['log_data_ip'] = new_ip
        
        with open(file_path, 'w') as f:
            json.dump(config, f, indent=2)
        
        return True
    except Exception as e:
        raise RuntimeError(f"更新配置文件时出错: {e}")

def generate_launch_description():

    mid360_custom_msg = LaunchConfiguration('mid360_custom_msg')
    declare_mid360_custom_msg = DeclareLaunchArgument('mid360_custom_msg', default_value='0', 
        description='0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format')

    local_ip = save_target_ip()
    print("local_ip:", local_ip)
    if local_ip:
        update_mid360_config(
            os.path.join(get_package_share_directory('mid360'), 'config', 'MID360_config.json'), 
            local_ip
        )
    
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params 
            + [{"xfer_format": mid360_custom_msg}],
    )

    # --- point_cloud2类型 转化为 laser_scan数据类型 --- #
    pcl2scan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', 'livox/lidar'),
                    ('scan', '/scan')],             # 话题名称重映射
        parameters=[{
            'target_frame': 'laser_link',            # 重映射的laser_scan数据发布坐标系
            'transform_tolerance': 0.01,
            'min_height': -0.05,                    # 激光雷达采集高度下限
            'max_height': 0.2,                      # 激光雷达采集高度上限
            'angle_min': -2.9,                      # -M_PI/2
            'angle_max': 2.9,                       # M_PI/2
            'angle_increment': 0.0087,              # M_PI/360.0
            'scan_time': 0.02,
            'range_min': 0.2,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0}],
        name='pointcloud_to_laserscan')
    
    ld = LaunchDescription()
    ld.add_action(declare_mid360_custom_msg)
    ld.add_action(livox_driver)
    # ld.add_action(pcl2scan_node)

    return ld