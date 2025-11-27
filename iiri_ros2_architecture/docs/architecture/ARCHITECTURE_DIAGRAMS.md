# IIRI 机器人系统详细架构图

**文档版本**: v1.0.0
**创建日期**: 2025-10-30
**作者**: 唐文浩
**说明**: 本文档包含完整的系统架构可视化图表 (Mermaid 格式)

---

## 目录

1. [系统整体部署架构](#1-系统整体部署架构)
2. [ROS2 五层架构详细图](#2-ros2-五层架构详细图)
3. [qr_wl → ros2_control 迁移架构](#3-qr_wl--ros2_control-迁移架构)
4. [网络通信和数据流](#4-网络通信和数据流)
5. [OTA 升级流程](#5-ota-升级流程)
6. [qr_chart 监控客户端架构](#6-qr_chart-监控客户端架构)
7. [CI/CD 流水线](#7-cicd-流水线)
8. [完整技术栈](#8-完整技术栈)

---

## 1. 系统整体部署架构

### 1.1 物理部署视图

```mermaid
graph TB
    subgraph "开发/监控端 (PC/Mac)"
        DEV["开发者工作站<br/>Ubuntu 22.04"]
        QR_CHART["qr_chart 监控客户端<br/>Qt6 应用程序<br/>实时数据可视化"]
        WEB_BROWSER["Web 浏览器<br/>Vue3 前端<br/>远程控制界面"]
    end

    subgraph "GitLab 服务器 (192.168.1.55)"
        GITLAB["GitLab 仓库"]
        CORE_REPO["core_layer<br/>仓库"]
        HW_REPO["hardware_layer<br/>仓库"]
        PERC_REPO["perception_layer<br/>仓库"]
        INTEL_REPO["intelligence_layer<br/>仓库"]
        APP_REPO["application_layer<br/>仓库"]

        GITLAB --> CORE_REPO
        GITLAB --> HW_REPO
        GITLAB --> PERC_REPO
        GITLAB --> INTEL_REPO
        GITLAB --> APP_REPO
    end

    subgraph "Harbor 镜像仓库 (192.168.1.93)"
        HARBOR["Harbor Registry"]
        X86_IMG["build_x86_ros2:latest<br/>ROS2 Humble + 编译环境"]
        ARM_IMG["build_arm_ros2:latest<br/>Jetson 优化镜像"]

        HARBOR --> X86_IMG
        HARBOR --> ARM_IMG
    end

    subgraph "Jenkins CI/CD (192.168.1.59:8081)"
        JENKINS["Jenkins Master"]
        BUILD_JOB["iiri-layered-build-ci<br/>自动构建任务<br/>每 2 小时触发"]

        JENKINS --> BUILD_JOB
    end

    subgraph "机器人设备端 (192.168.1.54 - Jetson Orin)"
        ROBOT_DEVICE["Jetson Orin AGX<br/>ARM64 Ubuntu 22.04"]

        subgraph "系统服务"
            SYSTEMD["systemd 服务管理"]
            IIRI_SERVICE["iiri-ros.service<br/>ROS2 系统主服务"]
            SYSTEMD --> IIRI_SERVICE
        end

        subgraph "ROS2 运行环境"
            ROS2_RUNTIME["/home/wl/autorun/iiri-ros/<br/>ROS2 Humble 运行时"]
            INSTALL_DIR["install/<br/>编译产物"]
            CONFIG_DIR["config/<br/>配置文件"]
            LAUNCH_SCRIPT["start_ros2.sh<br/>启动脚本"]

            ROS2_RUNTIME --> INSTALL_DIR
            ROS2_RUNTIME --> CONFIG_DIR
            ROS2_RUNTIME --> LAUNCH_SCRIPT
        end

        subgraph "OTA 更新系统"
            UPDATE_MGR["update_manager<br/>OTA 升级程序"]
            BACKUP_DIR["/var/backups/iiri/<br/>版本备份"]
            LOG_DIR["/var/log/update_manager/<br/>升级日志"]

            UPDATE_MGR --> BACKUP_DIR
            UPDATE_MGR --> LOG_DIR
        end

        subgraph "应用层服务"
            DEV_SERVER["dev_server<br/>HTTP/WebSocket API<br/>端口: 8080"]
            REMOTE_CTRL["remote_ctrl<br/>遥控节点"]
            RECORD["record<br/>数据记录"]
        end

        ROBOT_DEVICE --> SYSTEMD
        ROBOT_DEVICE --> ROS2_RUNTIME
        ROBOT_DEVICE --> UPDATE_MGR
        ROBOT_DEVICE --> DEV_SERVER
    end

    %% 网络连接
    DEV -->|vcstool import| GITLAB
    DEV -->|docker pull| HARBOR
    DEV -->|查看构建状态| JENKINS

    QR_CHART -->|TCP 20333<br/>实时数据流| DEV_SERVER
    WEB_BROWSER -->|HTTP/WS 8080<br/>控制命令| DEV_SERVER

    BUILD_JOB -->|拉取代码| GITLAB
    BUILD_JOB -->|使用镜像| HARBOR
    BUILD_JOB -->|SSH 部署| UPDATE_MGR

    UPDATE_MGR -->|更新| ROS2_RUNTIME
    IIRI_SERVICE -->|启动| ROS2_RUNTIME
    ROS2_RUNTIME -->|运行| DEV_SERVER

    style ROBOT_DEVICE fill:#e1f5ff
    style GITLAB fill:#fce4ec
    style HARBOR fill:#f3e5f5
    style JENKINS fill:#fff3e0
    style DEV fill:#e8f5e9
```

### 1.2 版本管理和部署策略

```mermaid
graph LR
    subgraph "版本控制 (Git)"
        MAIN_REPO["主仓库<br/>iiri_ros2_architecture<br/>commit: c118d41"]

        subgraph "子模块指针"
            PTR_CORE["core_layer @ 28122ad"]
            PTR_HW["hardware_layer @ 258eae4"]
            PTR_PERC["perception_layer @ 522a395"]
            PTR_INTEL["intelligence_layer @ 18521f1"]
            PTR_APP["application_layer @ 1f3a6a4"]
        end

        MAIN_REPO --> PTR_CORE
        MAIN_REPO --> PTR_HW
        MAIN_REPO --> PTR_PERC
        MAIN_REPO --> PTR_INTEL
        MAIN_REPO --> PTR_APP
    end

    subgraph "构建产物"
        TARBALL["iiri-ros-arm-c118d41.tar.gz<br/>完整部署包"]
        SHA256["iiri-ros-arm-c118d41.tar.gz.sha256<br/>校验文件"]

        TARBALL --> SHA256
    end

    subgraph "机器人端部署目录"
        DEPLOY_DIR["/home/wl/autorun/"]

        VER1["iiri-ros-arm-c118d41/<br/>当前版本 v1.0.0"]
        VER2["iiri-ros-arm-258eae4/<br/>上一版本"]
        VER3["iiri-ros-arm-24c7b89/<br/>更早版本"]
        SYMLINK["iiri-ros → iiri-ros-arm-c118d41/<br/>符号链接"]

        DEPLOY_DIR --> VER1
        DEPLOY_DIR --> VER2
        DEPLOY_DIR --> VER3
        DEPLOY_DIR --> SYMLINK

        SYMLINK -.指向.-> VER1
    end

    MAIN_REPO -->|Jenkins 构建| TARBALL
    TARBALL -->|OTA 升级| DEPLOY_DIR

    style MAIN_REPO fill:#e3f2fd
    style TARBALL fill:#fff9c4
    style VER1 fill:#c8e6c9
    style SYMLINK fill:#ffccbc
```

---

## 2. ROS2 五层架构详细图

### 2.1 完整五层架构组件图

```mermaid
graph TB
    subgraph "Layer 5: Application Layer (应用通信层)"
        APP_DEV["dev_server<br/>━━━━━━━━<br/>• HTTP API (CrowCpp)<br/>• WebSocket 服务<br/>• Vue3 前端<br/>• ROS2 Bridge"]
        APP_UPDATE["update_manager<br/>━━━━━━━━<br/>• OTA 升级管理<br/>• 版本回滚<br/>• 健康检查<br/>• 日志管理"]
        APP_REMOTE["remote_ctrl<br/>━━━━━━━━<br/>• WebSocket 控制<br/>• 手柄映射<br/>• 速度命令<br/>• 状态反馈"]
        APP_RECORD["record<br/>━━━━━━━━<br/>• rosbag2 录制<br/>• 数据回放<br/>• 时序同步"]
        APP_KEY["key_control<br/>━━━━━━━━<br/>• 键盘控制<br/>• 快捷键映射<br/>• 调试工具"]
    end

    subgraph "Layer 4: Intelligence Layer (智能导航层)"
        INT_NAV["navigation<br/>━━━━━━━━<br/>• SLAM 定位<br/>• 路径规划<br/>• 障碍物避障<br/>• HDL 全局定位"]
        INT_BT["bt_manager<br/>━━━━━━━━<br/>• 行为树引擎<br/>• 任务调度<br/>• 状态机管理<br/>• 智能决策"]
        INT_FOLLOW["smart_follow<br/>━━━━━━━━<br/>• 目标跟踪<br/>• 视觉伺服<br/>• 安全距离保持"]
        INT_XIAOZHI["xiaozhi<br/>━━━━━━━━<br/>• 语音助手<br/>• 自然语言理解<br/>• 任务执行"]
        INT_TRACKER["path_tracker<br/>━━━━━━━━<br/>• 路径跟踪<br/>• Ceres 优化<br/>• MPC 控制"]
    end

    subgraph "Layer 3: Perception Layer (感知处理层)"
        PERC_CAM["camera_ptz<br/>━━━━━━━━<br/>• PTZ 云台控制<br/>• 目标检测<br/>• 跟踪算法<br/>• 图像流发布"]
        PERC_SPEAKER["speaker<br/>━━━━━━━━<br/>• 音频播放<br/>• TTS 输出<br/>• 音效管理"]
        PERC_TTS["tts<br/>━━━━━━━━<br/>• 文本转语音<br/>• 语音合成<br/>• 多语言支持"]
        PERC_ASR["speech_recognition<br/>━━━━━━━━<br/>• 语音识别<br/>• 关键词检测<br/>• 噪声抑制"]
    end

    subgraph "Layer 2: Hardware Layer (硬件抽象层)"
        HW_QR["qr_control<br/>━━━━━━━━<br/>• QrSpiCanHardware<br/>  - SPI-CAN 驱动<br/>  - 12 关节控制<br/>• QrFsmController<br/>  - FSM 状态机<br/>  - 1000Hz 控制<br/>  - PD 控制器"]
        HW_BASE["robot_base<br/>━━━━━━━━<br/>• 差速/全向轮<br/>• 里程计<br/>• 速度控制<br/>• 底盘驱动"]
        HW_SENSOR["sensor/*<br/>━━━━━━━━<br/>• imu_node (IMU)<br/>• gamepad_node (手柄)<br/>• lidar (激光雷达)<br/>• camera (相机)"]
    end

    subgraph "Layer 1: Core Layer (核心基础层)"
        CORE_INTERFACE["interface<br/>━━━━━━━━<br/>• msg/qr/ (四足消息)<br/>• msg/arm/ (机械臂)<br/>• srv/ (服务定义)<br/>• action/ (动作定义)"]
        CORE_BRINGUP["system_bringup<br/>━━━━━━━━<br/>• 1_hardware.launch.py<br/>• 2_perception.launch.py<br/>• 3_intelligence.launch.py<br/>• 4_application.launch.py<br/>• qr_orin.launch.py"]
        CORE_THIRD["third_party<br/>━━━━━━━━<br/>• backward_ros<br/>  (堆栈回溯)"]
    end

    subgraph "ROS2 Humble 基础环境"
        ROS2_BASE["ROS2 Humble LTS<br/>━━━━━━━━<br/>• rclcpp/rclpy<br/>• ament_cmake<br/>• colcon build<br/>• DDS (FastDDS)"]
    end

    %% 层间依赖关系
    APP_DEV --> INT_NAV
    APP_DEV --> INT_BT
    APP_REMOTE --> INT_BT
    APP_RECORD --> INT_NAV

    INT_NAV --> PERC_CAM
    INT_BT --> PERC_ASR
    INT_FOLLOW --> PERC_CAM
    INT_XIAOZHI --> PERC_TTS

    PERC_CAM --> HW_SENSOR
    PERC_TTS --> HW_BASE

    HW_QR --> CORE_INTERFACE
    HW_BASE --> CORE_INTERFACE
    HW_SENSOR --> CORE_INTERFACE

    CORE_INTERFACE --> ROS2_BASE
    CORE_BRINGUP --> ROS2_BASE

    style APP_DEV fill:#ffebee
    style APP_UPDATE fill:#ffebee
    style INT_NAV fill:#e8eaf6
    style PERC_CAM fill:#e0f2f1
    style HW_QR fill:#fff3e0
    style CORE_INTERFACE fill:#f3e5f5
    style ROS2_BASE fill:#e8f5e9
```

### 2.2 ROS2 Overlay 构建流程

```mermaid
graph LR
    subgraph "构建环境"
        DOCKER["Docker 容器<br/>192.168.1.93/iiri/build_arm_ros2"]
    end

    subgraph "Layer 1: Core"
        CORE_SRC["src/core_layer/"]
        CORE_BUILD["build_arm_shared/core_layer/build/"]
        CORE_INSTALL["build_arm_shared/core_layer/install/"]
        CORE_SETUP["setup.bash<br/>(source ROS2 Humble)"]

        CORE_SRC -->|colcon build| CORE_BUILD
        CORE_BUILD --> CORE_INSTALL
        CORE_INSTALL --> CORE_SETUP
    end

    subgraph "Layer 2: Hardware"
        HW_SRC["src/hardware_layer/"]
        HW_BUILD["build_arm_shared/hardware_layer/build/"]
        HW_INSTALL["build_arm_shared/hardware_layer/install/"]
        HW_SETUP["setup.bash<br/>(overlay core_layer)"]

        HW_SRC -->|colcon build| HW_BUILD
        HW_BUILD --> HW_INSTALL
        HW_INSTALL --> HW_SETUP
    end

    subgraph "Layer 3: Perception"
        PERC_SRC["src/perception_layer/"]
        PERC_BUILD["build_arm_shared/perception_layer/build/"]
        PERC_INSTALL["build_arm_shared/perception_layer/install/"]
        PERC_SETUP["setup.bash<br/>(overlay hardware)"]

        PERC_SRC -->|colcon build| PERC_BUILD
        PERC_BUILD --> PERC_INSTALL
        PERC_INSTALL --> PERC_SETUP
    end

    subgraph "Layer 4: Intelligence"
        INT_SRC["src/intelligence_layer/"]
        INT_BUILD["build_arm_shared/intelligence_layer/build/"]
        INT_INSTALL["build_arm_shared/intelligence_layer/install/"]
        INT_SETUP["setup.bash<br/>(overlay perception)"]

        INT_SRC -->|colcon build| INT_BUILD
        INT_BUILD --> INT_INSTALL
        INT_INSTALL --> INT_SETUP
    end

    subgraph "Layer 5: Application"
        APP_SRC["src/application_layer/"]
        APP_BUILD["build_arm_shared/application_layer/build/"]
        APP_INSTALL["build_arm_shared/application_layer/install/"]
        APP_SETUP["setup.bash<br/>(overlay intelligence)"]

        APP_SRC -->|colcon build| APP_BUILD
        APP_BUILD --> APP_INSTALL
        APP_INSTALL --> APP_SETUP
    end

    DOCKER --> CORE_SRC
    CORE_SETUP -->|source| HW_SRC
    HW_SETUP -->|source| PERC_SRC
    PERC_SETUP -->|source| INT_SRC
    INT_SETUP -->|source| APP_SRC

    style DOCKER fill:#e3f2fd
    style CORE_INSTALL fill:#f3e5f5
    style HW_INSTALL fill:#fff3e0
    style PERC_INSTALL fill:#e0f2f1
    style INT_INSTALL fill:#e8eaf6
    style APP_INSTALL fill:#ffebee
```

---

## 3. qr_wl → ros2_control 迁移架构

### 3.1 架构对比图

```mermaid
graph TB
    subgraph "传统 qr_wl 架构 (单体应用)"
        QR_WL["qr_wl 可执行文件<br/>═══════════════<br/>15K 行 C++ 代码"]

        subgraph "qr_wl 内部模块"
            QR_BASELINE["baseline/<br/>消息传递框架"]
            QR_CONTROL["control/<br/>• StateManager<br/>• fsmStand/Walk/Lie<br/>• PD 控制算法"]
            QR_APP["app/<br/>• motion/ (SPI-CAN)<br/>• driver/ (IMU/手柄)<br/>• network/ (UDP 20333)<br/>• robot/ (状态管理)"]
            QR_PROJECT["project/<br/>main.cpp 主入口"]
        end

        QR_WL --> QR_BASELINE
        QR_WL --> QR_CONTROL
        QR_WL --> QR_APP
        QR_WL --> QR_PROJECT

        QR_ISSUES["问题:<br/>━━━━<br/>• 500Hz 控制频率<br/>• 8ms 延迟<br/>• 只支持实机<br/>• 内存泄漏<br/>• 难以扩展"]

        QR_WL -.存在.-> QR_ISSUES
    end

    subgraph "目标 ros2_control 架构 (模块化)"
        subgraph "Hardware Interface"
            RC_HW["QrSpiCanHardware<br/>━━━━━━━━━━━━<br/>• read() - 读取关节状态<br/>• write() - 写入力矩命令<br/>• SPI-CAN 驱动<br/>• 12 关节管理<br/>• 1000Hz 更新"]
        end

        subgraph "Controller"
            RC_CTRL["QrFsmController<br/>━━━━━━━━━━━━<br/>• update() - 1000Hz 控制循环<br/>• FSM 嵌入 (500Hz)<br/>• PD 控制器<br/>• 话题订阅/发布"]
        end

        subgraph "FSM (嵌入 Controller)"
            RC_FSM["StateManager<br/>━━━━━━━━<br/>• fsmStand.cpp<br/>• fsmWalk.cpp<br/>• fsmLie.cpp<br/>• Run() - 状态更新<br/>• GetJointTargets()"]
        end

        subgraph "ROS2 Integration"
            RC_TOPICS["ROS2 Topics<br/>━━━━━━━━<br/>• /joint_states (发布)<br/>• /cmd_vel (订阅)<br/>• /imu_data (订阅)"]
            RC_SERVICES["ROS2 Services<br/>━━━━━━━━<br/>• /set_run_state<br/>• /get_robot_status"]
        end

        RC_HW -->|state_interfaces| RC_CTRL
        RC_CTRL -->|command_interfaces| RC_HW
        RC_CTRL -->|嵌入| RC_FSM
        RC_CTRL <-->|发布/订阅| RC_TOPICS
        RC_CTRL <-->|调用/响应| RC_SERVICES

        RC_BENEFITS["优势:<br/>━━━━<br/>• 1000Hz 控制频率<br/>• 3ms 延迟<br/>• 支持 Gazebo/MuJoCo<br/>• 无内存泄漏<br/>• 易于扩展 RL/MPC"]

        RC_CTRL -.带来.-> RC_BENEFITS
    end

    QR_WL -.迁移.-> RC_HW
    QR_CONTROL -.重构.-> RC_CTRL
    QR_CONTROL -.移植.-> RC_FSM

    style QR_WL fill:#ffcdd2
    style QR_ISSUES fill:#ef9a9a
    style RC_HW fill:#c8e6c9
    style RC_CTRL fill:#a5d6a7
    style RC_FSM fill:#81c784
    style RC_BENEFITS fill:#66bb6a
```

### 3.2 控制回路详细流程

```mermaid
sequenceDiagram
    participant HW as QrSpiCanHardware<br/>(硬件接口)
    participant CM as Controller Manager<br/>(ros2_control)
    participant CTRL as QrFsmController<br/>(控制器)
    participant FSM as StateManager<br/>(FSM 状态机)
    participant TOPIC as ROS2 Topics<br/>(话题通信)

    Note over HW,TOPIC: ━━━━━ 1000Hz 控制循环 ━━━━━

    loop 每 1ms (1000Hz)
        CM->>HW: read() 读取硬件状态
        activate HW
        HW-->>CM: state_interfaces<br/>(关节角度/速度/力矩)
        deactivate HW

        CM->>CTRL: update(time, period)
        activate CTRL

        Note over CTRL: 读取实时命令指针
        CTRL->>TOPIC: readFromRT()
        TOPIC-->>CTRL: 高层命令<br/>(速度/状态切换)

        alt 500Hz 执行 (每 2ms)
            CTRL->>FSM: Run()
            activate FSM
            Note over FSM: 状态机更新<br/>Stand/Walk/Lie
            FSM-->>CTRL: GetJointTargets()
            deactivate FSM
        end

        Note over CTRL: PD 控制计算
        loop 12 个关节
            CTRL->>CTRL: τ = Kp*(q_des - q) + Kd*(dq_des - dq)
        end

        CTRL-->>CM: command_interfaces<br/>(力矩命令)
        deactivate CTRL

        CM->>HW: write() 写入命令
        activate HW
        HW->>HW: SPI-CAN 发送
        deactivate HW

        Note over HW,TOPIC: ━━━━━ 循环周期: 1ms ━━━━━
    end
```

---

## 4. 网络通信和数据流

### 4.1 完整通信架构

```mermaid
graph TB
    subgraph "开发/监控端"
        QRCHART["qr_chart<br/>━━━━━━━━<br/>Qt6 桌面应用<br/>实时监控"]
        WEBBROWSER["Web 浏览器<br/>━━━━━━━━<br/>Vue3 界面<br/>远程控制"]
    end

    subgraph "机器人端 - Application Layer"
        DEVSERVER["dev_server<br/>━━━━━━━━<br/>• CrowCpp HTTP (8080)<br/>• WebSocket 服务<br/>• IIRI SDK Bridge"]
        REMOTECTRL["remote_ctrl<br/>━━━━━━━━<br/>• WS 命令解析<br/>• ROS2 发布"]
    end

    subgraph "机器人端 - ROS2 DDS 通信"
        TOPICS_PUB["发布话题<br/>━━━━━━━━<br/>• /joint_states (500Hz)<br/>• /imu_data (500Hz)<br/>• /camera_image (30Hz)<br/>• /bt_status (10Hz)"]

        TOPICS_SUB["订阅话题<br/>━━━━━━━━<br/>• /cmd_vel (50Hz)<br/>• /goal_pose (1Hz)<br/>• /run_state_cmd"]

        SERVICES["ROS2 服务<br/>━━━━━━━━<br/>• /set_run_state<br/>• /trigger_behavior<br/>• /emergency_stop"]
    end

    subgraph "机器人端 - Hardware Layer"
        QRCONTROL["qr_control<br/>━━━━━━━━<br/>• 1000Hz 控制<br/>• 关节状态发布"]
        ROBOTBASE["robot_base<br/>━━━━━━━━<br/>• 速度控制<br/>• 里程计发布"]
        IMUNODE["imu_node<br/>━━━━━━━━<br/>• IMU 数据<br/>• 姿态估计"]
    end

    %% qr_chart 通信
    QRCHART <-->|TCP 20333<br/>IIRI 二进制协议<br/>2ms 间隔| DEVSERVER

    %% Web 通信
    WEBBROWSER <-->|HTTP/WS 8080<br/>JSON 消息| DEVSERVER

    %% dev_server 到 ROS2
    DEVSERVER -->|订阅| TOPICS_PUB
    DEVSERVER -.->|调用| SERVICES

    %% remote_ctrl 到 ROS2
    REMOTECTRL -->|发布| TOPICS_SUB

    %% Hardware 到 Topics
    QRCONTROL -->|发布| TOPICS_PUB
    ROBOTBASE -->|发布| TOPICS_PUB
    IMUNODE -->|发布| TOPICS_PUB

    %% Topics 到 Hardware
    TOPICS_SUB -->|订阅| QRCONTROL
    TOPICS_SUB -->|订阅| ROBOTBASE

    style QRCHART fill:#e1f5ff
    style WEBBROWSER fill:#e1f5ff
    style DEVSERVER fill:#ffebee
    style TOPICS_PUB fill:#e8f5e9
    style TOPICS_SUB fill:#fff3e0
    style SERVICES fill:#f3e5f5
```

### 4.2 实时数据流 (500Hz)

```mermaid
sequenceDiagram
    participant QR as qr_control<br/>(1000Hz)
    participant DDS as ROS2 DDS<br/>(FastDDS)
    participant DEV as dev_server<br/>(ROS2 Bridge)
    participant SDK as IIRI SDK<br/>(TCP Server)
    participant CLIENT as qr_chart<br/>(TCP Client)

    Note over QR,CLIENT: ━━━━━ 每 2ms 数据周期 ━━━━━

    loop 每 2ms (500Hz)
        QR->>QR: 读取 12 关节状态<br/>(q, dq, tau)
        QR->>DDS: 发布 /joint_states<br/>sensor_msgs/JointState

        DDS->>DEV: DDS 订阅回调
        activate DEV

        DEV->>DEV: 转换为 IIRI 协议<br/>LegInfoSet 结构

        DEV->>SDK: SDK 数据包组装
        deactivate DEV

        SDK->>CLIENT: TCP 发送<br/>[Header|LegInfo|ImuInfo|...]

        CLIENT->>CLIENT: 协议解析<br/>debugWatch
        CLIENT->>CLIENT: 更新 DataManager
        CLIENT->>CLIENT: UI 刷新<br/>QCustomPlot
    end

    Note over QR,CLIENT: ━━━━━ 端到端延迟: ~5ms ━━━━━
```

### 4.3 端口和协议映射

```mermaid
graph LR
    subgraph "网络端口"
        PORT_20333["TCP 20333<br/>━━━━━━━━<br/>IIRI SDK 服务<br/>qr_chart 连接"]
        PORT_8080["HTTP/WS 8080<br/>━━━━━━━━<br/>dev_server API<br/>Web 界面"]
        PORT_22["SSH 22<br/>━━━━━━━━<br/>OTA 升级<br/>远程管理"]
        PORT_DDS["UDP 7400-7500<br/>━━━━━━━━<br/>ROS2 DDS<br/>节点间通信"]
    end

    subgraph "协议类型"
        PROTO_BIN["二进制协议<br/>━━━━━━━━<br/>• LegInfoSet<br/>• ImuInfoSet<br/>• ArmInfoSet<br/>高效率,低延迟"]
        PROTO_JSON["JSON 协议<br/>━━━━━━━━<br/>• WebSocket 消息<br/>• HTTP API<br/>易于调试"]
        PROTO_ROS2["ROS2 消息<br/>━━━━━━━━<br/>• sensor_msgs<br/>• geometry_msgs<br/>• std_msgs<br/>标准化"]
    end

    PORT_20333 --> PROTO_BIN
    PORT_8080 --> PROTO_JSON
    PORT_DDS --> PROTO_ROS2

    style PORT_20333 fill:#e3f2fd
    style PORT_8080 fill:#f3e5f5
    style PORT_DDS fill:#e8f5e9
```

---

## 5. OTA 升级流程

### 5.1 完整升级状态机

```mermaid
stateDiagram-v2
    [*] --> CREATED: 上传更新包

    CREATED --> PREPARING: 开始升级

    state PREPARING {
        [*] --> 解压更新包
        解压更新包 --> SHA256校验
        SHA256校验 --> GPG签名验证
        GPG签名验证 --> [*]
    }

    PREPARING --> RUNNING: 验证通过
    PREPARING --> FAILED: 验证失败

    state RUNNING {
        [*] --> 停止systemd服务
        停止systemd服务 --> 创建完整备份
        创建完整备份 --> 复制新文件
        复制新文件 --> 更新symlink
        更新symlink --> 启动服务
        启动服务 --> 健康检查
        健康检查 --> [*]
    }

    RUNNING --> SUCCESS: 健康检查通过
    RUNNING --> FAILED: 任一步骤失败

    FAILED --> ROLLBACK: 自动触发回滚

    state ROLLBACK {
        [*] --> 停止当前服务
        停止当前服务 --> 恢复备份文件
        恢复备份文件 --> 恢复symlink
        恢复symlink --> 重启服务
        重启服务 --> [*]
    }

    ROLLBACK --> ROLLBACK_SUCCESS: 回滚成功
    ROLLBACK --> ROLLBACK_FAILED: 回滚失败

    SUCCESS --> [*]
    ROLLBACK_SUCCESS --> [*]
    ROLLBACK_FAILED --> [*]: 需人工介入

    note right of PREPARING
        耗时: ~30秒
        SHA256 校验文件完整性
        可选 GPG 签名验证
    end note

    note right of RUNNING
        耗时: ~2分钟
        关键步骤有重试机制
        健康检查超时 30秒
    end note

    note right of ROLLBACK
        耗时: ~1分钟
        自动保留最近3个备份
        快速恢复到上一版本
    end note
```

### 5.2 OTA 升级详细流程

```mermaid
sequenceDiagram
    participant USER as 开发者/CI<br/>(触发升级)
    participant SSH as SSH 连接<br/>(远程执行)
    participant UPD as update_manager<br/>(主程序)
    participant FILE as FileVerifier<br/>(校验模块)
    participant BACKUP as BackupManager<br/>(备份模块)
    participant SYS as SystemdManager<br/>(服务管理)
    participant HEALTH as HealthChecker<br/>(健康检查)
    participant STATUS as StatusManager<br/>(状态日志)

    USER->>SSH: scp 上传更新包<br/>iiri-ros-arm-<commit>.tar.gz
    USER->>SSH: ssh 执行 OTA 命令

    SSH->>UPD: ./update_manager<br/>--task-id xxx<br/>--package /tmp/xxx.tar.gz<br/>--app-type ros2

    activate UPD
    UPD->>STATUS: setState(CREATED)
    STATUS-->>STATUS: 写日志文件<br/>/var/log/update_manager/

    Note over UPD,STATUS: ━━━━━ PREPARING 阶段 ━━━━━

    UPD->>STATUS: setState(PREPARING)
    UPD->>UPD: 解压到临时目录<br/>/tmp/update_xxx/

    UPD->>FILE: verifySHA256WithFile()
    activate FILE
    FILE->>FILE: 读取 .sha256 文件
    FILE->>FILE: calculateSHA256(tar.gz)
    FILE-->>UPD: 校验结果: PASS ✓
    deactivate FILE

    alt GPG 签名存在
        UPD->>FILE: verifyGPGSignature()
        FILE-->>UPD: 签名验证: PASS ✓
    end

    Note over UPD,STATUS: ━━━━━ RUNNING 阶段 ━━━━━

    UPD->>STATUS: setState(RUNNING)<br/>setProgress(10%)

    UPD->>SYS: stopService("iiri-ros.service")
    activate SYS
    SYS->>SYS: systemctl stop iiri-ros
    SYS-->>UPD: 服务已停止
    deactivate SYS

    UPD->>STATUS: setProgress(30%)
    UPD->>BACKUP: createBackup()
    activate BACKUP
    BACKUP->>BACKUP: cp -r /home/wl/autorun/iiri-ros/<br/>→ /var/backups/iiri/backup-<timestamp>/
    BACKUP-->>UPD: 备份完成
    deactivate BACKUP

    UPD->>STATUS: setProgress(50%)
    UPD->>UPD: 复制新文件<br/>cp -r /tmp/update_xxx/*<br/>→ /home/wl/autorun/iiri-ros-<commit>/

    UPD->>STATUS: setProgress(60%)
    UPD->>UPD: 更新 symlink<br/>ln -snf iiri-ros-<commit> iiri-ros

    UPD->>STATUS: setProgress(70%)
    UPD->>SYS: startService("iiri-ros.service")
    activate SYS
    SYS->>SYS: systemctl start iiri-ros
    SYS-->>UPD: 服务已启动
    deactivate SYS

    UPD->>STATUS: setProgress(80%)
    UPD->>HEALTH: performHealthCheck()
    activate HEALTH

    loop 健康检查 (最多30秒)
        HEALTH->>HEALTH: checkRosTopics()<br/>检查 /joint_states 等话题
        HEALTH->>HEALTH: checkServices()<br/>检查 ROS2 服务可用性
        HEALTH->>HEALTH: checkProcesses()<br/>检查关键进程运行

        alt 所有检查通过
            HEALTH-->>UPD: 健康检查: PASS ✓
        else 超时或失败
            HEALTH-->>UPD: 健康检查: FAIL ✗
        end
    end
    deactivate HEALTH

    alt 健康检查通过
        UPD->>STATUS: setState(SUCCESS)<br/>setProgress(100%)
        UPD->>BACKUP: cleanup() 清理旧备份
        UPD-->>SSH: 升级成功!

    else 健康检查失败
        Note over UPD,STATUS: ━━━━━ ROLLBACK 流程 ━━━━━

        UPD->>STATUS: setState(FAILED)<br/>setError("健康检查失败")
        UPD->>STATUS: setState(ROLLBACK)

        UPD->>SYS: stopService("iiri-ros.service")

        UPD->>BACKUP: rollback()
        activate BACKUP
        BACKUP->>BACKUP: 恢复备份文件<br/>cp -r /var/backups/iiri/backup-xxx/*<br/>→ /home/wl/autorun/iiri-ros/
        BACKUP->>BACKUP: 恢复 symlink
        BACKUP-->>UPD: 回滚完成
        deactivate BACKUP

        UPD->>SYS: startService("iiri-ros.service")

        UPD->>HEALTH: performHealthCheck()

        alt 回滚后健康
            UPD->>STATUS: setState(ROLLBACK_SUCCESS)
            UPD-->>SSH: 升级失败,已回滚到上一版本
        else 回滚也失败
            UPD->>STATUS: setState(ROLLBACK_FAILED)<br/>setError("需人工介入")
            UPD-->>SSH: 严重错误,需人工修复!
        end
    end

    deactivate UPD

    SSH-->>USER: OTA 升级结果
```

### 5.3 升级管理器模块架构

```mermaid
graph TB
    subgraph "update_manager 核心模块"
        MAIN["main.cpp<br/>━━━━━━━━<br/>• 参数解析<br/>• 配置加载<br/>• 流程编排"]

        subgraph "6 大管理器"
            MOD_STATUS["StatusManager<br/>━━━━━━━━<br/>• 状态机管理<br/>• 进度跟踪<br/>• 日志记录<br/>• JSON 持久化"]

            MOD_FILE["FileVerifier<br/>━━━━━━━━<br/>• SHA256 计算<br/>• 哈希值校验<br/>• GPG 签名验证"]

            MOD_BACKUP["BackupManager<br/>━━━━━━━━<br/>• 创建完整备份<br/>• 验证备份完整性<br/>• 回滚恢复<br/>• 清理旧备份"]

            MOD_SYSTEMD["SystemdManager<br/>━━━━━━━━<br/>• 停止服务<br/>• 启动服务<br/>• 查询状态<br/>• 重启管理"]

            MOD_HEALTH["HealthChecker<br/>━━━━━━━━<br/>• ROS2 话题检查<br/>• 服务可用性<br/>• 进程存活检测<br/>• 综合健康评估"]

            MOD_CMD["CommandExecutor<br/>━━━━━━━━<br/>• Shell 命令执行<br/>• 超时控制<br/>• 输出捕获<br/>• 错误处理"]
        end

        MAIN --> MOD_STATUS
        MAIN --> MOD_FILE
        MAIN --> MOD_BACKUP
        MAIN --> MOD_SYSTEMD
        MAIN --> MOD_HEALTH

        MOD_STATUS --> MOD_CMD
        MOD_FILE --> MOD_CMD
        MOD_BACKUP --> MOD_CMD
        MOD_SYSTEMD --> MOD_CMD
        MOD_HEALTH --> MOD_CMD
    end

    subgraph "数据结构"
        TYPES["update_types.hpp<br/>━━━━━━━━<br/>• UpdateState 枚举<br/>• UpdateTask 结构<br/>• UpdateConfig 配置"]
    end

    subgraph "外部接口"
        LOG_FILE["/var/log/update_manager/<br/>update_<taskId>.log<br/>详细日志文件"]

        STATUS_FILE["/var/run/update_status.json<br/>实时状态 JSON<br/>进度和错误信息"]

        BACKUP_DIR["/var/backups/iiri/<br/>backup-<timestamp>/<br/>版本备份目录"]
    end

    MAIN --> TYPES
    MOD_STATUS --> LOG_FILE
    MOD_STATUS --> STATUS_FILE
    MOD_BACKUP --> BACKUP_DIR

    style MAIN fill:#ffebee
    style MOD_STATUS fill:#e3f2fd
    style MOD_FILE fill:#f3e5f5
    style MOD_BACKUP fill:#fff3e0
    style MOD_SYSTEMD fill:#e0f2f1
    style MOD_HEALTH fill:#e8eaf6
    style MOD_CMD fill:#fce4ec
```

---

## 6. qr_chart 监控客户端架构

### 6.1 四层架构详细图

```mermaid
graph TB
    subgraph "UI Layer - Qt6 Widgets"
        MAINWIN["MainWindow<br/>━━━━━━━━<br/>主窗口容器<br/>Tab 页面管理"]

        TAB_QR["Tab: Quadruped<br/>━━━━━━━━<br/>chartQr<br/>四足机器人可视化"]
        TAB_ARM["Tab: Robotic Arm<br/>━━━━━━━━<br/>chartArm<br/>机械臂可视化"]
        TAB_HUMAN["Tab: Humanoid<br/>━━━━━━━━<br/>chartHuman<br/>人形机器人可视化"]

        UI_READ["uiReadDatabase<br/>━━━━━━━━<br/>历史数据查看<br/>时间序列分析"]
        UI_WRITE["uiWriteDatabase<br/>━━━━━━━━<br/>数据记录控制<br/>会话管理"]
        UI_SEARCH["uiSearchRobot<br/>━━━━━━━━<br/>机器人发现<br/>网络扫描"]

        MAINWIN --> TAB_QR
        MAINWIN --> TAB_ARM
        MAINWIN --> TAB_HUMAN
        MAINWIN --> UI_READ
        MAINWIN --> UI_WRITE
        MAINWIN --> UI_SEARCH
    end

    subgraph "Visualization Layer - QCustomPlot"
        CHARTWIDGET["chartWidget<br/>━━━━━━━━<br/>图表组件基类<br/>实时绘图引擎"]

        PLOT_JOINT["关节曲线<br/>━━━━━━━━<br/>• 角度 (q)<br/>• 速度 (dq)<br/>• 力矩 (tau)<br/>实时 + 期望值"]

        PLOT_IMU["IMU 曲线<br/>━━━━━━━━<br/>• 姿态角 (RPY)<br/>• 角速度<br/>• 加速度<br/>3D 可视化"]

        PLOT_ARM["机械臂曲线<br/>━━━━━━━━<br/>• 关节位置<br/>• 末端轨迹<br/>• 示教路径"]

        TABDATA["chartTabData<br/>━━━━━━━━<br/>分页数据管理<br/>缓冲区调度"]

        CHARTWIDGET --> PLOT_JOINT
        CHARTWIDGET --> PLOT_IMU
        CHARTWIDGET --> PLOT_ARM
        CHARTWIDGET --> TABDATA

        TAB_QR --> CHARTWIDGET
        TAB_ARM --> CHARTWIDGET
        TAB_HUMAN --> CHARTWIDGET
    end

    subgraph "Data Management Layer"
        DATAMGR["DataManager<br/>━━━━━━━━<br/>全局单例<br/>共享内存空间"]

        subgraph "数据结构"
            DS_LEG["LegInfoSet<br/>━━━━━━━━<br/>• q[12] 关节角度<br/>• dq[12] 速度<br/>• tau[12] 力矩<br/>• q_des/dq_des/tau_des"]

            DS_IMU["ImuInfoSet<br/>━━━━━━━━<br/>• quaternion[4]<br/>• gyroscope[3]<br/>• accelerometer[3]<br/>• rpy[3]"]

            DS_ARM["ArmInfoSet<br/>━━━━━━━━<br/>• position[16]<br/>• velocity[16]<br/>• effort[16]"]

            DS_HUMAN["HumanMotorSet<br/>━━━━━━━━<br/>• angle[32]<br/>• torque[32]<br/>• temperature[32]"]
        end

        DATABASE["Database (SQLite)<br/>━━━━━━━━<br/>• Session 表<br/>• TimeSeries 表<br/>• dbVersion/ Schema"]

        DATAMGR --> DS_LEG
        DATAMGR --> DS_IMU
        DATAMGR --> DS_ARM
        DATAMGR --> DS_HUMAN
        DATAMGR --> DATABASE

        CHARTWIDGET --> DATAMGR
        UI_READ --> DATABASE
        UI_WRITE --> DATABASE
    end

    subgraph "Network Layer - IIRI SDK"
        DEBUGCLIENT["DebugClient<br/>━━━━━━━━<br/>TCP 客户端<br/>Port: 20333"]

        DEBUGWATCH["debugWatch<br/>━━━━━━━━<br/>协议解析器<br/>回调系统"]

        PROTOCOL["IIRI 二进制协议<br/>━━━━━━━━<br/>• Header (时间戳)<br/>• LegInfoSet<br/>• ImuInfoSet<br/>• ArmInfoSet<br/>• 2ms 间隔 (500Hz)"]

        DEBUGCLIENT --> DEBUGWATCH
        DEBUGWATCH --> PROTOCOL
        PROTOCOL --> DATAMGR

        UI_SEARCH -.扫描网络.-> DEBUGCLIENT
    end

    subgraph "机器人端 (192.168.1.54)"
        SDK_SERVER["dev_server<br/>━━━━━━━━<br/>IIRI SDK 服务端<br/>TCP Server 20333"]
    end

    DEBUGCLIENT <-->|TCP 连接<br/>实时数据流| SDK_SERVER

    style MAINWIN fill:#e3f2fd
    style CHARTWIDGET fill:#e8f5e9
    style DATAMGR fill:#fff3e0
    style DEBUGCLIENT fill:#f3e5f5
    style SDK_SERVER fill:#ffebee
```

### 6.2 实时数据更新流程

```mermaid
sequenceDiagram
    participant ROBOT as dev_server<br/>(机器人端)
    participant TCP as TCP Socket<br/>(20333)
    participant CLIENT as DebugClient<br/>(TCP 客户端)
    participant WATCH as debugWatch<br/>(协议解析)
    participant MGR as DataManager<br/>(数据管理)
    participant PLOT as QCustomPlot<br/>(图表绘制)
    participant UI as MainWindow<br/>(UI 界面)

    Note over ROBOT,UI: ━━━━━ 每 2ms 数据周期 (500Hz) ━━━━━

    loop 每 2ms
        ROBOT->>ROBOT: 订阅 ROS2 Topics<br/>/joint_states<br/>/imu_data

        ROBOT->>ROBOT: 封装 IIRI 协议包<br/>[Header|Leg|Imu|Arm|...]

        ROBOT->>TCP: send() 二进制数据
        TCP->>CLIENT: recv() 接收数据

        CLIENT->>WATCH: 数据包解析
        activate WATCH

        WATCH->>WATCH: 解析 Header<br/>(时间戳,数据类型)

        alt LegInfoSet 数据
            WATCH->>MGR: updateLegInfo(data)
            MGR->>MGR: legInfoBuffer.push(data)
        end

        alt ImuInfoSet 数据
            WATCH->>MGR: updateImuInfo(data)
            MGR->>MGR: imuInfoBuffer.push(data)
        end

        alt ArmInfoSet 数据
            WATCH->>MGR: updateArmInfo(data)
            MGR->>MGR: armInfoBuffer.push(data)
        end

        deactivate WATCH

        WATCH->>PLOT: dataReady() 信号

        activate PLOT
        PLOT->>MGR: getLegInfo()
        MGR-->>PLOT: 最新数据

        PLOT->>PLOT: addData(time, value)<br/>更新曲线数据

        PLOT->>PLOT: replot()<br/>重绘图表
        deactivate PLOT

        PLOT->>UI: repaint 信号
        UI->>UI: UI 刷新
    end

    Note over ROBOT,UI: ━━━━━ 刷新频率: 500Hz ━━━━━<br/>━━━━━ 端到端延迟: ~10ms ━━━━━
```

---

## 7. CI/CD 流水线

### 7.1 Jenkins 自动构建流程

```mermaid
graph TB
    START([Jenkins 触发器])

    subgraph "触发条件"
        CRON["定时触发<br/>H */2 * * *<br/>(每2小时)"]
        GIT_PUSH["Git Push<br/>主分支提交"]
        MANUAL["手动触发<br/>Jenkins UI"]
    end

    START --> CRON
    START --> GIT_PUSH
    START --> MANUAL

    CRON --> STAGE1
    GIT_PUSH --> STAGE1
    MANUAL --> STAGE1

    subgraph "Stage 1: 代码导入"
        STAGE1["vcstool 导入<br/>━━━━━━━━"]
        CLONE["克隆主仓库<br/>iiri_ros2_architecture"]
        VCS_IMPORT["vcs import src < .repos<br/>导入 5 个分层仓库"]
        GIT_INFO["生成 Git 版本信息<br/>commit hash + tag"]

        STAGE1 --> CLONE
        CLONE --> VCS_IMPORT
        VCS_IMPORT --> GIT_INFO
    end

    subgraph "Stage 2: Docker 环境"
        STAGE2["Docker 准备<br/>━━━━━━━━"]
        DOCKER_LOGIN["登录 Harbor<br/>192.168.1.93"]
        DOCKER_PULL["拉取镜像<br/>build_x86_ros2:latest"]
        DOCKER_RUN["启动构建容器<br/>挂载工作空间"]

        GIT_INFO --> STAGE2
        STAGE2 --> DOCKER_LOGIN
        DOCKER_LOGIN --> DOCKER_PULL
        DOCKER_PULL --> DOCKER_RUN
    end

    subgraph "Stage 3: 分层编译"
        STAGE3["Colcon 构建<br/>━━━━━━━━"]
        BUILD_CORE["Layer 1: core_layer<br/>source /opt/ros/humble/setup.bash"]
        BUILD_HW["Layer 2: hardware_layer<br/>overlay core_layer"]
        BUILD_PERC["Layer 3: perception_layer<br/>overlay hardware_layer"]
        BUILD_INT["Layer 4: intelligence_layer<br/>overlay perception_layer"]
        BUILD_APP["Layer 5: application_layer<br/>overlay intelligence_layer"]

        DOCKER_RUN --> STAGE3
        STAGE3 --> BUILD_CORE
        BUILD_CORE --> BUILD_HW
        BUILD_HW --> BUILD_PERC
        BUILD_PERC --> BUILD_INT
        BUILD_INT --> BUILD_APP
    end

    subgraph "Stage 4: 打包归档"
        STAGE4["Artifact 生成<br/>━━━━━━━━"]
        TAR_CREATE["压缩构建产物<br/>tar -czf iiri-ros-x86-<commit>.tar.gz"]
        SHA_GEN["生成 SHA256<br/>sha256sum > .tar.gz.sha256"]
        ARCHIVE["归档到 Jenkins<br/>/var/jenkins_home/workspace/"]

        BUILD_APP --> STAGE4
        STAGE4 --> TAR_CREATE
        TAR_CREATE --> SHA_GEN
        SHA_GEN --> ARCHIVE
    end

    subgraph "Stage 5: 测试 (可选)"
        STAGE5["自动化测试<br/>━━━━━━━━"]
        UNIT_TEST["单元测试<br/>colcon test"]
        INT_TEST["集成测试<br/>system_bringup 启动"]
        PERF_TEST["性能测试<br/>ROS2 topic 频率"]

        ARCHIVE --> STAGE5
        STAGE5 --> UNIT_TEST
        UNIT_TEST --> INT_TEST
        INT_TEST --> PERF_TEST
    end

    subgraph "Stage 6: 部署"
        STAGE6["远程部署<br/>━━━━━━━━"]
        SCP_UPLOAD["上传到机器人<br/>scp → 192.168.1.54:/tmp/"]
        TRIGGER_OTA["触发 OTA 升级<br/>ssh update_manager"]
        HEALTH_CHECK["验证部署<br/>健康检查"]

        PERF_TEST --> STAGE6
        STAGE6 --> SCP_UPLOAD
        SCP_UPLOAD --> TRIGGER_OTA
        TRIGGER_OTA --> HEALTH_CHECK
    end

    HEALTH_CHECK --> SUCCESS([构建成功])
    HEALTH_CHECK -.失败.-> FAIL([构建失败])

    SUCCESS --> NOTIFY["通知<br/>━━━━━━━━<br/>• Email<br/>• 企业微信<br/>• Slack"]
    FAIL --> NOTIFY

    style START fill:#e8f5e9
    style SUCCESS fill:#c8e6c9
    style FAIL fill:#ffcdd2
    style NOTIFY fill:#fff9c4
```

### 7.2 Harbor 镜像管理

```mermaid
graph TB
    subgraph "Harbor 镜像仓库 (192.168.1.93)"
        HARBOR["Harbor Registry<br/>━━━━━━━━<br/>私有 Docker 仓库"]

        subgraph "iiri 项目"
            PROJ_X86["build_x86_ros2<br/>━━━━━━━━"]
            PROJ_ARM["build_arm_ros2<br/>━━━━━━━━"]
        end

        HARBOR --> PROJ_X86
        HARBOR --> PROJ_ARM

        subgraph "x86 镜像版本"
            X86_LATEST["latest<br/>最新开发版<br/>自动更新"]
            X86_V143["v1.4.3<br/>稳定版本<br/>ROS2 Humble"]
            X86_V142["v1.4.2<br/>上一版本<br/>备份保留"]
        end

        subgraph "ARM 镜像版本"
            ARM_LATEST["latest<br/>最新开发版<br/>Jetson 优化"]
            ARM_V142["v1.4.2<br/>稳定版本<br/>NVIDIA Runtime"]
            ARM_V141["v1.4.1<br/>上一版本<br/>备份保留"]
        end

        PROJ_X86 --> X86_LATEST
        PROJ_X86 --> X86_V143
        PROJ_X86 --> X86_V142

        PROJ_ARM --> ARM_LATEST
        PROJ_ARM --> ARM_V142
        PROJ_ARM --> ARM_V141
    end

    subgraph "镜像内容"
        CONTENT["基础镜像: Ubuntu 22.04<br/>━━━━━━━━<br/>• ROS2 Humble 完整环境<br/>• vcstool 0.3.0<br/>• colcon build 工具<br/>• 编译依赖<br/>  - OpenCV 4.x<br/>  - PCL 1.12<br/>  - Eigen 3.4<br/>  - CUDA 11.8 (GPU 版)<br/>• 交叉编译工具链<br/>• NVIDIA Runtime (ARM)"]
    end

    subgraph "使用场景"
        USE_DEV["开发环境<br/>━━━━━━━━<br/>docker pull<br/>本地编译"]
        USE_CI["CI/CD 构建<br/>━━━━━━━━<br/>Jenkins 自动拉取<br/>自动化编译"]
        USE_TEST["测试环境<br/>━━━━━━━━<br/>容器化测试<br/>环境隔离"]
    end

    X86_LATEST -.使用.-> USE_DEV
    ARM_LATEST -.使用.-> USE_CI
    X86_V143 -.使用.-> USE_TEST

    CONTENT -.包含于.-> X86_LATEST
    CONTENT -.包含于.-> ARM_LATEST

    style HARBOR fill:#e3f2fd
    style CONTENT fill:#fff3e0
    style USE_CI fill:#e8f5e9
```

---

## 8. 完整技术栈

### 8.1 系统技术栈全景图

```mermaid
graph TB
    subgraph "操作系统层"
        OS_DEV["开发端<br/>Ubuntu 22.04 LTS<br/>Linux Kernel 6.8"]
        OS_ROBOT["机器人端<br/>Ubuntu 22.04 LTS<br/>Jetson Linux 35.x"]
    end

    subgraph "容器化层"
        DOCKER["Docker 20.10+<br/>━━━━━━━━<br/>容器运行时"]
        HARBOR["Harbor 2.x<br/>━━━━━━━━<br/>镜像仓库"]

        DOCKER --> HARBOR
    end

    subgraph "ROS2 中间件层"
        ROS2["ROS2 Humble LTS<br/>━━━━━━━━"]
        DDS["FastDDS<br/>DDS 实现"]
        RCLCPP["rclcpp<br/>C++ 客户端库"]
        RCLPY["rclpy<br/>Python 客户端库"]

        ROS2 --> DDS
        ROS2 --> RCLCPP
        ROS2 --> RCLPY
    end

    subgraph "构建系统层"
        COLCON["colcon 0.12+<br/>━━━━━━━━<br/>ROS2 构建工具"]
        CMAKE["CMake 3.16+<br/>━━━━━━━━<br/>构建配置"]
        AMENT["ament_cmake<br/>━━━━━━━━<br/>ROS2 扩展"]

        COLCON --> CMAKE
        COLCON --> AMENT
    end

    subgraph "版本管理层"
        GIT["Git 2.x<br/>━━━━━━━━<br/>版本控制"]
        VCSTOOL["vcstool 0.3.0<br/>━━━━━━━━<br/>多仓库管理"]
        GITLAB["GitLab<br/>━━━━━━━━<br/>代码托管"]

        GIT --> GITLAB
        VCSTOOL --> GIT
    end

    subgraph "CI/CD 层"
        JENKINS["Jenkins 2.x<br/>━━━━━━━━<br/>自动化构建"]
        PIPELINE["Jenkinsfile<br/>━━━━━━━━<br/>流水线定义"]

        JENKINS --> PIPELINE
    end

    subgraph "应用框架层"
        ROS2CTRL["ros2_control<br/>━━━━━━━━<br/>实时控制框架"]
        QT6["Qt6 6.2+<br/>━━━━━━━━<br/>GUI 框架"]
        VUE3["Vue3 3.x<br/>━━━━━━━━<br/>Web 前端"]

        ROS2CTRL --> ROS2
        QT6 -.客户端.-> QT6
        VUE3 -.Web界面.-> VUE3
    end

    subgraph "核心库层"
        EIGEN["Eigen 3.4<br/>━━━━━━━━<br/>线性代数"]
        OPENCV["OpenCV 4.x<br/>━━━━━━━━<br/>计算机视觉"]
        PCL["PCL 1.12<br/>━━━━━━━━<br/>点云处理"]
        CERES["Ceres 2.x<br/>━━━━━━━━<br/>优化库"]
        BOOST["Boost 1.74<br/>━━━━━━━━<br/>C++ 扩展"]
        OPENSSL["OpenSSL 3.x<br/>━━━━━━━━<br/>加密库"]

        ROS2CTRL --> EIGEN
        ROS2 --> OPENCV
        ROS2 --> PCL
        ROS2CTRL -.可选.-> CERES
    end

    subgraph "网络通信层"
        TCP["TCP/IP<br/>━━━━━━━━<br/>可靠传输"]
        WS["WebSocket<br/>━━━━━━━━<br/>实时通信"]
        HTTP["HTTP/HTTPS<br/>━━━━━━━━<br/>Web API"]
        SSH["SSH<br/>━━━━━━━━<br/>安全远程"]

        DDS --> TCP
        VUE3 --> WS
        VUE3 --> HTTP
    end

    subgraph "数据存储层"
        SQLITE["SQLite 3.x<br/>━━━━━━━━<br/>嵌入式数据库"]
        ROSBAG["rosbag2<br/>━━━━━━━━<br/>时序数据"]
        JSON["JSON<br/>━━━━━━━━<br/>配置文件"]
    end

    subgraph "硬件驱动层"
        SPI["SPI-CAN<br/>━━━━━━━━<br/>电机通信"]
        IMU_DRV["IMU Driver<br/>━━━━━━━━<br/>惯性传感器"]
        LIDAR_DRV["LiDAR Driver<br/>━━━━━━━━<br/>激光雷达"]
        CAM_DRV["Camera Driver<br/>━━━━━━━━<br/>相机驱动"]
    end

    %% 层间依赖
    OS_DEV --> DOCKER
    OS_ROBOT --> ROS2
    DOCKER --> ROS2
    ROS2 --> COLCON
    COLCON --> GIT
    GIT --> JENKINS
    JENKINS --> HARBOR

    ROS2 --> ROS2CTRL
    ROS2CTRL --> SPI
    ROS2 --> IMU_DRV
    ROS2 --> LIDAR_DRV
    ROS2 --> CAM_DRV

    QT6 --> SQLITE
    ROS2 --> ROSBAG

    style OS_ROBOT fill:#e8f5e9
    style ROS2 fill:#e3f2fd
    style ROS2CTRL fill:#fff3e0
    style QT6 fill:#f3e5f5
    style JENKINS fill:#fff9c4
```

### 8.2 依赖关系矩阵

```mermaid
graph LR
    subgraph "语言和工具链"
        CPP["C++17<br/>━━━━━━━━<br/>主要开发语言"]
        PYTHON["Python 3.10<br/>━━━━━━━━<br/>脚本和工具"]
        TS["TypeScript<br/>━━━━━━━━<br/>Web 前端"]
        BASH["Bash<br/>━━━━━━━━<br/>Shell 脚本"]
    end

    subgraph "编译器"
        GCC["GCC 11.x<br/>━━━━━━━━<br/>C++ 编译器"]
        CLANG["Clang 14.x<br/>━━━━━━━━<br/>备用编译器"]
    end

    subgraph "包管理"
        APT["apt<br/>━━━━━━━━<br/>系统包管理"]
        NPM["npm<br/>━━━━━━━━<br/>Node.js 包"]
        PIP["pip<br/>━━━━━━━━<br/>Python 包"]
    end

    CPP --> GCC
    CPP --> CLANG
    PYTHON --> PIP
    TS --> NPM

    APT --> GCC
    APT --> PYTHON

    style CPP fill:#e3f2fd
    style PYTHON fill:#fff3e0
    style TS fill:#f3e5f5
```

---

## 总结

本文档提供了 IIRI 机器人系统的完整可视化架构图,涵盖:

1. **系统部署**: 物理拓扑、网络连接、版本管理
2. **ROS2 架构**: 五层分层、Overlay 机制、组件依赖
3. **qr_wl 迁移**: 架构对比、控制回路、性能提升
4. **网络通信**: 协议栈、数据流、端口映射
5. **OTA 升级**: 状态机、详细流程、模块架构
6. **qr_chart**: 四层架构、实时数据、UI 组件
7. **CI/CD**: Jenkins 流水线、Harbor 镜像管理
8. **技术栈**: 完整的软件栈和依赖关系

所有图表使用 Mermaid 格式,支持在 GitHub、GitLab、VS Code 等平台直接渲染。

**查看方式**:
- GitHub/GitLab: 自动渲染
- VS Code: 安装 "Markdown Preview Mermaid Support" 插件
- 在线工具: https://mermaid.live/

---

**文档版本**: v1.0.0
**最后更新**: 2025-10-30
**维护者**: 唐文浩
