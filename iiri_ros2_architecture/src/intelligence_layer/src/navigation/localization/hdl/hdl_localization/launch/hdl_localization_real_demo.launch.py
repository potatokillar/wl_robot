#############################################################################

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
import os

from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():

    # Importtant, required arguments
    points_topic = LaunchConfiguration('points_topic', default='livox/lidar')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id', default='laser_link') 
    imu_topic = LaunchConfiguration('imu_topic', default='livox/imu')

    # arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # optional arguments
    use_imu = LaunchConfiguration('use_imu', default='false')
    invert_imu_acc = LaunchConfiguration('invert_imu_acc', default='false')
    invert_imu_gyro = LaunchConfiguration('invert_imu_gyro', default='false')
    use_global_localization = LaunchConfiguration('use_global_localization', default='true')
    enable_robot_odometry_prediction = LaunchConfiguration('enable_robot_odometry_prediction', default='false')
    robot_odom_frame_id = LaunchConfiguration('robot_odom_frame_id', default='odom')
    plot_estimation_errors = LaunchConfiguration('plot_estimation_errors', default='false')

    urdf = os.path.join(get_package_share_directory('hdl_localization'), 'urdf', 'mid360.urdf')

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
                    {'global_map_name': "horizon_map-2025-06-03-21:06:33.pcd"},
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
                    ('/velodyne_points', points_topic), 
                    ('/gpsimu_driver/imu_data', imu_topic)
                ],
                parameters=[
                    {'odom_child_frame_id': odom_child_frame_id},
                    {'use_imu': use_imu},
                    {'invert_acc': invert_imu_acc},
                    {'invert_gyro': invert_imu_gyro},
                    {'cool_time_duration': 2.0},
                    {'enable_robot_odometry_prediction': enable_robot_odometry_prediction},
                    {'robot_odom_frame_id': robot_odom_frame_id},
                    # <!-- available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D-->
                    {'reg_method': 'NDT_OMP'},
                    {'ndt_neighbor_search_method': 'DIRECT7'},
                    {'ndt_neighbor_search_radius': 1.0},        #  暂时不需要看，好像是cuda加速才会用到的参数
                    {'ndt_resolution': 1.0},                    #  best: 1.0 ;   
                    {'downsample_resolution': 0.22},            #  best: 0.22 ; 当较近障碍物较为密集时，该值要设置的小一点，从而不至于忽略远处墙壁特征 
                    {'specify_init_pose': True},
                    {'init_pos_x': 0.0},
                    {'init_pos_y': 0.0},
                    {'init_pos_z': 0.0},
                    {'init_ori_w': 1.0},
                    {'init_ori_x': 0.0},
                    {'init_ori_y': 0.0},
                    {'init_ori_z': 0.0},
                    {'use_global_localization': use_global_localization},
                    {'use_sim_time': use_sim_time}
                ]
            )
        ],
        output='screen',   
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
    
    record_traj_node = Node(
        package='record_traj',
        executable='record_traj_node',
        parameters=[
            os.path.join(
                get_package_share_directory('record_traj'),
                'config',
                'record_traj.yaml'
            )
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('hdl_localization'),
                'rviz',
                'real.rviz'
            )
        ],
        output='screen'
    )

    declare_points_topic = DeclareLaunchArgument('points_topic', default_value=points_topic, description='Point cloud topic name')
    declare_odom_child_frame_id = DeclareLaunchArgument('odom_child_frame_id', default_value=odom_child_frame_id, description='Odom frame id')
    declare_imu_topic = DeclareLaunchArgument('imu_topic', default_value=imu_topic)
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    declare_use_imu = DeclareLaunchArgument('use_imu', default_value=use_imu)
    declare_invert_imu_acc = DeclareLaunchArgument('invert_imu_acc', default_value=invert_imu_acc)
    declare_invert_imu_gyro = DeclareLaunchArgument('invert_imu_gyro', default_value=invert_imu_gyro)
    declare_use_global_localization = DeclareLaunchArgument('use_global_localization', default_value=use_global_localization)
    declare_enable_robot_odometry_prediction = DeclareLaunchArgument('enable_robot_odometry_prediction', default_value=enable_robot_odometry_prediction)
    declare_robot_odom_frame_id = DeclareLaunchArgument('robot_odom_frame_id', default_value=robot_odom_frame_id)
    declare_plot_estimation_errors = DeclareLaunchArgument('plot_estimation_errors', default_value=plot_estimation_errors)

    ld = LaunchDescription()
    ld.add_action(declare_points_topic)
    ld.add_action(declare_odom_child_frame_id)
    ld.add_action(declare_imu_topic)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_imu)
    ld.add_action(declare_invert_imu_acc)
    ld.add_action(declare_invert_imu_gyro)
    ld.add_action(declare_use_global_localization)
    ld.add_action(declare_enable_robot_odometry_prediction)
    ld.add_action(declare_robot_odom_frame_id)
    ld.add_action(declare_plot_estimation_errors)

    ld.add_action(robot_state_publisher)
    ld.add_action(sensor_launch)
    ld.add_action(container)   
    ld.add_action(hdl_global_localization_launch)
    ld.add_action(record_traj_node)
    ld.add_action(rviz_node)

    return ld