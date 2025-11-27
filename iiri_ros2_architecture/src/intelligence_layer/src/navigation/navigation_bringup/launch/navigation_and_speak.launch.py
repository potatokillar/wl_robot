#############################################################################

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():

    # arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # optional arguments
    use_global_localization = LaunchConfiguration('use_global_localization', default='true')

    urdf = os.path.join(
        get_package_share_directory('navigation_bringup'), 
        'urdf', 
        'odom_to_base.urdf'
    )
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    
    robot_state_publisher = Node(
        name = 'robot_state_publisher',
        package = 'robot_state_publisher',
        executable='robot_state_publisher',
        output = 'screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description' : robot_description
        }]
    )

    # --- 启动传感器 --- #
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sensor'),
                'launch',
                'sensor.launch.py')),
        launch_arguments={
            'sensor_type' : 'mid360',
            'mid360_custom_msg' : '0'
        }.items()
    )

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::GlobalmapServerNodelet',
                name='GlobalmapServerNodelet',
                parameters=[
                    {'global_map_parent_directory': "map"},
                    {'global_map_name': "horizon_map-2025-06-16-19:10:04.pcd"},
                    {'convert_utm_to_local': True},
                    {'downsample_resolution': 0.1},  # 使用该参数对全局地图进行过滤，将过滤后的点云地图作为点云配准的全局地图。
                    {'use_sim_time': use_sim_time}
                ]
            ),
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::HdlLocalizationNodelet',
                name='HdlLocalizationNodelet',
                remappings=[
                    ('/velodyne_points', 'livox/lidar'), 
                    ('/gpsimu_driver/imu_data', 'livox/imu')
                ],
                parameters=[
                    {'robot_odom_frame_id': 'odom'},
                    {'odom_child_frame_id': 'base_link'},
                    {'use_imu': False},
                    {'invert_acc': False},
                    {'invert_gyro': False},
                    {'cool_time_duration': 2.0},
                    {'enable_robot_odometry_prediction': False},
                    # <!-- available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D-->
                    {'reg_method': 'NDT_OMP'},
                    {'ndt_neighbor_search_method': 'DIRECT7'},
                    {'ndt_neighbor_search_radius': 1.0},        #  暂时不需要看，好像是cuda加速才会用到的参数
                    {'ndt_resolution': 1.0},                    #  best: 1.0 ;   
                    {'downsample_resolution': 0.22},            #  best: 0.22 ; 当较近障碍物较为密集时，该值要设置的小一点，从而不至于忽略远处墙壁特征 
                    {'specify_init_pose': True},
                    {'init_pos_x': 14.5},
                    {'init_pos_y': 29.5},
                    {'init_pos_z': 0.15},
                    {'init_ori_w': -0.021},
                    {'init_ori_x': 0.0090},
                    {'init_ori_y': 0.0134},
                    {'init_ori_z': 0.9996},
                    {'use_global_localization': use_global_localization},
                    {'use_sim_time': use_sim_time}
                ]
            ),
            ComposableNode(
                package='path_tracker',
                plugin='path_tracker::PathTrackerNode',
                name="path_tracker_node",
                parameters=[
                    os.path.join(
                        get_package_share_directory('navigation_bringup'),
                        'config',
                        'path_tracker_node.yaml'
                    ),
                    {'use_sim_time': use_sim_time},
                ]
            ),
            ComposableNode(
                package='navigation_lifecycle_manager',
                plugin='navigation_lifecycle_manager::LifecycleManager',
                name="lifecycle_manager",
                parameters=[
                    {'auto_start': True},
                    {'node_names': ['path_tracker_node']},
                    {'use_sim_time': use_sim_time},
                ]
            ),
        ],
        output='screen',   
    )

    simple_path_tracker_api_node = Node(
        package="navigation_api",
        executable="simple_path_tracker_api_node"
    )

    # include hdl_global_localization launch file
    hdl_global_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hdl_global_localization'),
                'launch',
                'hdl_global_localization.launch.py'
            ),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_global_localization),
    )

    navigation_and_speak_key_node = Node(
        package='navigation_key',
        executable='navigation_and_speak_key_node',
        parameters=[
            os.path.join(
                get_package_share_directory('navigation_bringup'),
                'config',
                'navigation_and_speak_key.yaml'
            )
        ]
    )

    collision_detect_node = Node(
        package='extra',
        executable='collision_detect',
        remappings=[
            ('point_cloud_in', '/livox/lidar'),
        ],
        parameters=[
            os.path.join(
                get_package_share_directory('navigation_bringup'),
                'config',
                'collision_detect_node.yaml'
            )
        ]
    )

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    declare_use_global_localization = DeclareLaunchArgument('use_global_localization', default_value=use_global_localization)

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_global_localization)

    ld.add_action(robot_state_publisher)
    ld.add_action(sensor_launch)
    ld.add_action(container)   
    ld.add_action(hdl_global_localization_launch)
    ld.add_action(simple_path_tracker_api_node)
    ld.add_action(navigation_and_speak_key_node)
    ld.add_action(collision_detect_node)

    return ld