# 使用说明

### 1. 文件目录架构概述

![image-20250428114331168](/home/wl/.config/Typora/typora-user-images/image-20250428114331168.png)

1. **extra**：该目录主要包含一些中间转发节点（后期大概率不需要这部分，不太规范），例如将世界坐标系下的无人机的避障运动控制指令转化到机器人当前位姿坐标系下用来控制四足机器人。
2. **localization**：主要包含定位功能核心算法模块（hdl）及一些自己添加的辅助功能（记录机器人位置轨迹，可能会作为后面用来巡检预设路径）；
3. **map**：主要包含建图核心算法模块（fast_lio）及一些辅助功能（fast_lio建图只会以开始建图的前几帧的位姿作为全局地图的坐标系，并不会使用imu数据将其与重力方向对齐，辅助功能中目前包含将全局地图与水平对齐功能）；

### 2. 主要模块使用方法

#### 2.1 **mapping建图**

###### **启动方法**

```bash
# 进入docker容器
./docker.sh 运行并进入docker容器
# source ros2功能包环境
source ./build_x86/install/setup.bash
# 启动demo launch文件
ros2 launch fast_lio mapping.launch.py
```

###### **使用方法**

1. 移动激光雷达保证地图的完整性，通过下面的命令来呼叫保存原始点云地图：

   ```bash
   ros2 service call /map_save std_srvs/srv/Trigger
   ```

2. 也可以利用horizon_map辅助功能包来保存水平对齐后的点云地图：

   1. 将小车装到小车上，推动小车到两两距离较远的三个位置，分别呼叫一次总共三次记录机器人服务：

      ```bash
      ros2 service call /record_point std_srvs/srv/Trigger
      ```

   2. 通过下面的命令呼叫保存点云地图服务，通过计算上面记录的三点所在三角平面的法向量计算当前三角形平面旋转至水平的旋转矩阵，将发布的点云与水平方向对齐并保存：

      ```bash
      ros2 service call /save_horizontal_map std_srvs/srv/Trigger
      ```

###### **注意事项**

1. 建图**launch文件**中的配置文件参数(**config_file参数**)要选择正确，一般是**mid360.yaml**文件；将**horizon_map_node**中接收的**地图话题名称**以及**里程计话题名称**在**launch文件**中可能需要使用remapping映射为正确的话题名称，一般不用修改；

   **路径：**`src/navigation/map/FAST_LIO/launch/mapping.launch.py`

   | 参数名        | 参数含义                                                     | 常用数值    |
   | ------------- | ------------------------------------------------------------ | ----------- |
   | 'config_file' | 参数文件名称（参数文件都在fast_lio功能包config目录下）       | mid360.yaml |
   | 'sensor_type' | 激光雷达数据源型号，使用livox传感器可以接收livox自定义点云消息类型建图更准确 | 'mid360'    |

2. 修改配置文件中的接收的雷达点云型号，保存地图文件的父目录名称以及文件名称，一般那会死生成在FAST_LIO功能包源码目录的PCD目录下命名为map；

   **路径：**`src/navigation/map/FAST_LIO/config/mid360.yaml`

   | 参数名              | 参数含义                                                     | 常用数值       |
   | ------------------- | ------------------------------------------------------------ | -------------- |
   | map_file_parent_directory | 地图文件的父目录名称（指功能包目录下存储地图的文件夹名称）   | "PCD"          |
   | map_file_name     | 地图文件名称前缀，后缀添加时间戳                             | "map"          |
   | lid_topic           | 雷达点云话题名称                                             | "/livox/lidar" |
   | imu_topic           | imu数据话题名称                                              | "/livox/imu"   |
   | lidar_type          | 雷达点云数据类型(1. livox_custom；4. 普通的ros2点云消息sensor_msgs/msg/PointCloud；其余类型暂时用不到) | 1              |

3. 注意将mid360驱动发布的消息类型改为livox_custom类型这样每个点云都有时间戳，可以通过imu来进行点云畸变矫正。不然就是正常的ros2点云消息，对应上面的文件中的lidar_type就要修改为4。

   **路径：**`src/sensor/lidar/mid360/launch/mid360.launch.py`

   | 参数名       | 参数含义                                                     | 常用数值     |
   | ------------ | ------------------------------------------------------------ | ------------ |
   | xfer_format  | ros2驱动发布的雷达点云数据类型（0- PointCloud2；1- livox_custom） | 1            |
   | publish_freq | 雷达话题发布频率，频率越高点云密度越小，可能不利于点云定位   | 9.0          |
   | frame_id     | 雷达数据发布的坐标系名称                                     | 'laser_link' |
   | imu_frame_id | imu数据发布的坐标系名称                                      | 'imu_link'   |

#### 2.2 localization定位

###### **启动方法**

```shell
# 进入docker容器
./docker.sh 运行并进入docker容器
# source ros2功能包环境
source ./build_x86/install/setup.bash
# 启动demo launch文件
ros2 launch hdl_localization hdl_localization_real.launch.py
```

###### 使用方法

1. 可以通过ros2 rviz2界面中的2D Pose Estimate指定初始位置，否则默认就是在launch文件中设置好的init_pos初始位姿数值：

   ![image-20250428163350814](/home/wl/.config/Typora/typora-user-images/image-20250428163350814.png)

2. 通过下面的命令开始记录机器人的轨迹信息：

   ```bash
   ros2 service call /start_record std_srvs/srv/Trigger
   ```

3. 通过下面的命令将机器人的轨迹信息保存为csv文件

   ```bash
   ros2 service call /save_traj std_srvs/srv/Trigger
   ```

4. 通过下面的命令停止记录机器人的轨迹信息

   ```bash
   ros2 service call /save_traj std_srvs/srv/Trigger
   ```

###### 注意事项

1. hdl_localization_real.launch.py文件参数配置正确

   **路径：**`src/navigation/localization/hdl/hdl_localization/launch/hdl_localization_real.launch.py`

   | 参数名                                           | 参数含义                                                     | 常用数值                          |
   | ------------------------------------------------ | ------------------------------------------------------------ | --------------------------------- |
   | 'odom_child_frame_id'                            | 里程计子坐标系名称，一般就是激光雷达连杆                     | 'laser_link'                      |
   | 'globalmap_pcd'                                  | 加载全局地图的路径                                           | 'fast_lio/PCD/map.pcd'            |
   | 'use_imu'                                        | 定位算法是否使用imu数据                                      | false/true                        |
   | 'use_global_localization'                        | 是否打开全局定位功能（该功能暂时不稳定）                     | 'true'                            |
   | urdf                                             | 发布各个连杆之间位姿关系的urdf文件路径                       | hdl_localization/urdf/mid360.urdf |
   | 'sensor_type'                                    | 发布雷达点云的sensor驱动型号（只要能发出正常ros2点云消息类型的就可以） | 'mid360'                          |
   | 'GlobalmapServerNodelet'/'downsample_resolution' | 过滤全局地图的分辨率                                         | 0.1                               |
   | '/velodyne_points'                               | 定位接收的实时点云话题名                                     | '/livox/lidar'                    |
   | '/gpsimu_driver/imu_data'                        | 定位接收的实时imu话题名                                      | '/livox/imu'                      |
   | 'odom_child_frame_id'                            | 里程计子坐标系名称，一般是激光雷达连杆位姿（定位算法更新的位姿坐标系） | 'laser_link'                      |
   | 'robot_odom_frame_id'                            | 里程计坐标系，机器人在地图中的初始位姿（定位算法的初始位姿） | 'odom'                            |
   | 'ndt_resolution'                                 | 计算正态分布参数的网格大小                                   | 1.0                               |
   | 'HdlLocalizationNodelet'/'downsample_resolution' | 定位算法实时输入点云的降采样分别率                           | 0.22                              |
   | 'specify_init_pose'                              | 是否指定初始位姿                                             | True                              |
   | init_pos                                         | 初始位置                                                     | 0.0，0.0，0.0                     |
   | init_ori                                         | 初始姿态（四元数）                                           | 1.0, 0.0, 0.0, 0.0                |


#### 2.3 extra工具包

该功能包主要包含一些避障导航的中间节点、临时功能以及一些文件操作的小工具库

###### 2.3.1 collision_detect.cpp（临时功能）: 

**主要功能：**

  用于获得四足附近八个方向的最近点坐标与距离信息，并发布给其他模块用于避障。

**主要参数：**`wl_ros/src/navigation/extra/launch/collision_detect.launch.py`

| 参数名称              | 参数说明                                                   | 常用参数               |
| --------------------- | ---------------------------------------------------------- | ---------------------- |
| 'sensor_type'         | 雷达传感器型号(具体可以参考sensor目录下说明)               | 'rs16'、'mid360'、'yd' |
| 'horizon_thresh_dist' | 雷达点云的最大距离，大于该距离的点将被过滤                 | '10.0'                 |
| 'horizon_thresh_ang'  | 雷达点云的俯仰角限制，与雷达xy平面超过该角度的点的将被过滤 | '2.0'                  |
| 'point_cloud_in'      | 输入的点云消息话题名称                                     | '/rslidar_points'      |

###### 2.3.2 ego_planner方案的中间节点

1. qr_cmd_node.cpp（中间节点）: 

- **主要功能：**

  - 接收来自**traj_server节点**的类型为quadrotor_msgs::msg::PositionCommand的**无人机控制指令**消息。

  - 接收来自**定位模块**传入的当前传感器位姿消息。

  - 将**无人机控制指令**根据**当前的位姿**转化到**传感器坐标系**下，取其中的**x、y指令**并将**z轴速度赋为0**，**z轴角速度指令**直接使用**无人机的偏航速度**。赋值到**Twist消息**中通过**cmd_vel话题**发送给四足机器人控制器。

2. qr_odom_node.cpp（中间节点）: 

- **主要功能：**

  - 接收来自**定位模块**传入的**当前传感器位姿**消息。

  - 接收来自**cmd_vel话题**的四足速度控制指令直接作为**当前的速度状态**。

  - 由于规划器ego-planner需要接收的是机器人**世界坐标系下的速度以及位置状态**，通过**定位模块获取到的位姿**将**cmd_vel速度方向**转化到**世界坐标系下**与当前位姿结合作为此时的机器人状态odom发布到ros2节点中。

###### 2.3.3 resource_tools.cpp：resource文件路径操作小工具

​	**主要功能：**当前较大体积的数据文件（pcd点云文件、csv机器人轨迹文件）集中放到wl_ros外层的resource文件目录下，方便读取与写入操作的统一，通过在别的C++代码中包含resource_tools.hpp头文件，可以方便的获取到想要写入文件的路径以及读取文件的路径。

​	**使用方法：**

1. 首先在你的功能包中通过下面代码添加extra工具包依赖

   ```cmake
   find_package(extra)
   ament_target_dependencies(<your_target> extra)
   ```

2. 注意package.xml中同样要添加依赖，像正常的添加ros2依赖一样

   ```xml
   <depend>extra</depend>
   ```

3. C++代码中

   ```C++
   // 包含头文件
   #include "extra/resource_tool.hpp"
   
   // 实例化一个ResourceTools对象，通过该对象调用方法
   ResourceTool resource_tools;
   
   // 当你在创建文件将数据写入一个文件前，想要获取resource下创建该文件的完整路径时，你可以
   std::string pcl_file_path = resource_tools.create_file(<file_parent_dir_name>, <file_name>);
   // 或者通过下面的方法获取带有时间戳信息的完整文件路径
   std::string pcl_file_path = resource_tools.create_file_with_timestamp(file_parent_dir_name, file_name, ".pcd");
   //*** 上面的<file_parent_dir_name>代表你的文件想要放置的父目录名称，比如想将地图点云文件放到resource/map文件夹下，你就可以将<file_parent_dir_name>参数填成map，<file_name>参数则是你想要创建的文件名。***//
   
   // 当想要读出一个已存在的文件时，你可以使用下面的方法
   std::string read_file_path;
   bool file_exist = resource_tools.get_file_path(<file_parent_dir_name>, <file_name>, read_file_path);
   //*** 上面的<file_parent_dir_name>代表你想要读取文件所在的resource目录下的父目录名称，比如你想要读取的地图点云文件在resource/map文件夹下，你可以将<file_parent_dir_name>参数填成map，<file_name>参数则是你想要读取文件的文件名。***//
   ```
   

#### 2.4 ego_planner demo + qr_wl + simulation_all

###### 启动方法

```
# 打开simulation_all 并启动小型四足仿真
./start.sh qr sz 
# 打开qr_wl 注意要将config/00-base.toml 中的 useGamepad 标志位改为false，关闭手柄功能
./start.sh qr sz
# 打开wl_ros，先要让四足机器人的站立
./start.sh stand
# 四足机器人切换到行走模式后，ctrl-c关闭，启动ego-planner规划器以及四足控制指令转发节点
./start.sh ego_pub_to_qr
```

###### 重点修改参数

- **文件路径**: `wl_ros/src/bringup/launch/ego_pub_to_qr.launch.py`

**参数介绍**：

| 参数名称       | 参数解释             | 常用参数                   |
| -------------- | -------------------- | -------------------------- |
| 'motionCfg'    | 手柄 IP 配置文件     | qr_local.yaml 、qr.yaml    |
| 'drone_cmd_in' | 输入控制指令话题映射 | 'drone_0_planning/pos_cmd' |
| 'qr_odom_in    | 输入里程计话题映射   | 'drone_0_visual_slam/odom' |

- **文件路径**：

`wl_ros/src/navigation/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py`

**参数介绍** : 

| 参数名称                   | 参数解释                                          | 常用参数                       |
| -------------------------- | ------------------------------------------------- | ------------------------------ |
| 'use_dynamic'              | 仿真飞行是否使用动力学                            | False (目前设置为True不能飞行) |
| 'flight_type'              | 目标点设置模式选择                                | 1：手动；2：预设               |
| 'planning_horizon'         | 规划局部路径的与无人机之间的最远距离              | 7.5                            |
| 'point_num'                | 预设目标点数量                                    | 4                              |
| 'point_x/y/z'              | 预设目标点坐标（点的数量要和上面的point_num对齐） | /                              |
| 'fsm/realworld_experiment' | 是否真实样机上飞行标志位                          | False(由于是仿真)              |

