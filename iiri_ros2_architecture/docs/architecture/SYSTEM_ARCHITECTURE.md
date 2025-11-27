# IIRI 机器人系统整体架构

**文档版本**: v1.0.0
**创建日期**: 2025-10-30
**作者**: 唐文浩
**项目**: IIRI 智能机器人系统

---

## 执行摘要

本文档描述了 IIRI (西湖大学智能产业研究院) 机器人系统的完整技术架构,整合了以下四个核心子系统:

1. **iiri_ros2_architecture** - ROS2 Humble 五层分层架构主系统
2. **ros2_control 迁移方案** - qr_wl 四足机器人控制器往 ROS2 Control 框架迁移
3. **qr_chart** - Qt6 桌面监控和数据可视化客户端
4. **OTA 更新系统** - update_manager 远程无缝升级方案

**系统特点**:
- 分层解耦的模块化设计
- 支持多种机器人平台 (四足/机械臂/人形)
- 实时控制 (1000Hz) + 高级智能功能
- 完整的 OTA 升级能力
- 跨平台监控和调试工具

---

## 目录

- [1. 系统全景架构](#1-系统全景架构)
- [2. ROS2 五层架构详解](#2-ros2-五层架构详解)
- [3. qr_wl → ROS2 Control 迁移架构](#3-qr_wl--ros2-control-迁移架构)
- [4. qr_chart 监控客户端](#4-qr_chart-监控客户端)
- [5. OTA 更新系统](#5-ota-更新系统)
- [6. 网络通信架构](#6-网络通信架构)
- [7. 部署和运维架构](#7-部署和运维架构)
- [8. 未来演进路线图](#8-未来演进路线图)

---

## 1. 系统全景架构

### 1.1 整体视图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          IIRI 机器人系统全景                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌───────────────────────┐                    ┌──────────────────────────┐  │
│  │    开发/监控端         │                    │     机器人设备端          │  │
│  │  (PC/Mac/Linux/Web)   │                    │   (Jetson Orin / Pi)     │  │
│  └───────────────────────┘                    └──────────────────────────┘  │
│            │                                              │                   │
│            │  ┌──────────────────────────┐              │                   │
│            │  │      监控通道             │              │                   │
│            ▼  ▼                          ▼              ▼                   │
│   ┌─────────────────┐          ┌──────────────────────────────┐            │
│   │   qr_chart      │  TCP     │  dev_server (Application)    │            │
│   │   Qt6 监控客户端 │◄────────►│  - HTTP API (8080)           │            │
│   │                 │  20333   │  - WebSocket (实时数据)       │            │
│   │  ├─ QCustomPlot │          │  - Vue3 前端                  │            │
│   │  ├─ SQLite DB   │          └──────────────────────────────┘            │
│   │  └─ IIRI SDK    │                      │                                │
│   └─────────────────┘                      │                                │
│                                             ▼                                │
│                              ┌──────────────────────────────────────┐       │
│                              │  iiri_ros2_architecture (ROS2)      │       │
│                              │  ═══════════════════════════════════ │       │
│                              │                                      │       │
│                              │  ┌─ Application Layer (应用层)      │       │
│                              │  │  ├─ dev_server (Web/RPC)         │       │
│                              │  │  ├─ update_manager (OTA) ◄───┐   │       │
│                              │  │  ├─ remote_ctrl (遥控)       │   │       │
│                              │  │  └─ record (数据记录)         │   │       │
│                              │  │                              │   │       │
│                              │  ┌─ Intelligence Layer (智能层) │   │       │
│                              │  │  ├─ navigation (导航)        │   │       │
│                              │  │  ├─ bt_manager (行为树)      │   │       │
│                              │  │  ├─ smart_follow (跟随)      │   │       │
│                              │  │  └─ xiaozhi (语音助手)       │   │       │
│                              │  │                              │   │       │
│                              │  ┌─ Perception Layer (感知层)   │   │       │
│                              │  │  ├─ camera_ptz (云台相机)    │   │       │
│                              │  │  ├─ speaker (扬声器)         │   │       │
│                              │  │  ├─ tts (语音合成)           │   │       │
│                              │  │  └─ speech_recognition (语音) │   │       │
│                              │  │                              │   │       │
│                              │  ┌─ Hardware Layer (硬件层)     │   │       │
│                              │  │  ├─ qr_control (四足控制)    │   │       │
│                              │  │  │  ├─ QrSpiCanHardware      │   │       │
│                              │  │  │  └─ QrFsmController       │   │       │
│                              │  │  ├─ robot_base (底盘)        │   │       │
│                              │  │  └─ sensor (传感器)          │   │       │
│                              │  │                              │   │       │
│                              │  └─ Core Layer (核心层)         │   │       │
│                              │     ├─ interface (消息定义)     │   │       │
│                              │     └─ system_bringup (启动)    │   │       │
│                              │                                  │   │       │
│                              └──────────────────────────────────┘   │       │
│                                             ▲                       │       │
│                                             │                       │       │
│                                    ┌────────┴─────────┐            │       │
│                                    │   OTA 升级通道    │            │       │
│                                    │  (SSH/HTTP)      │◄───────────┘       │
│                                    └──────────────────┘                     │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 关键设计原则

| 原则 | 说明 | 实现方式 |
|------|------|---------|
| **分层解耦** | 清晰的架构边界，便于维护 | ROS2 五层架构，每层独立仓库 |
| **实时性能** | 确保控制回路的确定性延迟 | ros2_control 框架，1000Hz 控制频率 |
| **可扩展性** | 支持新硬件和算法的快速集成 | 标准化接口，plugin 机制 |
| **可观测性** | 实时监控和历史数据分析 | qr_chart + ROS2 topics/services |
| **可维护性** | OTA 升级，远程诊断 | update_manager + dev_server |
| **容错性** | 故障检测和自动恢复 | 健康检查 + 回滚机制 |

---

## 2. ROS2 五层架构详解

### 2.1 架构总览

```
┌─────────────────────────────────────────────────────────────────────┐
│                       ROS2 五层分层架构                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Layer 5: Application Layer (应用通信层)                             │
│  ════════════════════════════════════════════                        │
│  • dev_server        - Web 界面和 RPC 服务器                         │
│  • update_manager    - OTA 升级管理器                                │
│  • remote_ctrl       - WebSocket 远程控制                            │
│  • record            - 数据记录和回放                                 │
│  • key_control       - 键盘控制                                      │
│  依赖: Intelligence Layer                                            │
│  编译频率: 低 (业务逻辑变更)                                          │
│                                                                       │
├───────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Layer 4: Intelligence Layer (智能导航层)                            │
│  ═══════════════════════════════════════                             │
│  • navigation        - 自主导航 (SLAM/定位/路径规划)                  │
│  • bt_manager        - 行为树任务管理                                 │
│  • smart_follow      - 智能跟随                                      │
│  • xiaozhi           - 语音助手                                      │
│  • path_tracker      - 路径跟踪 (可选 Ceres 优化)                    │
│  依赖: Perception Layer                                              │
│  编译频率: 中 (算法优化)                                              │
│                                                                       │
├───────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Layer 3: Perception Layer (感知处理层)                              │
│  ═══════════════════════════════════                                 │
│  • camera_ptz        - PTZ 云台相机控制                               │
│  • speaker           - 扬声器输出                                     │
│  • tts               - 文本转语音                                     │
│  • speech_recognition - 语音识别                                     │
│  依赖: Hardware Layer                                                │
│  编译频率: 中 (感知算法调整)                                          │
│                                                                       │
├───────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Layer 2: Hardware Layer (硬件抽象层)                                │
│  ═══════════════════════════════════                                 │
│  • qr_control        - 四足机器人控制 (ros2_control)                 │
│    ├─ QrSpiCanHardware  - SPI-CAN 硬件接口                           │
│    └─ QrFsmController   - FSM 状态机控制器 (1000Hz)                  │
│  • robot_base        - 移动底盘驱动                                   │
│  • sensor            - 传感器驱动集合                                 │
│    ├─ imu_node       - IMU 惯性测量单元                              │
│    ├─ gamepad_node   - 手柄输入                                      │
│    └─ lidar          - 激光雷达                                      │
│  依赖: Core Layer                                                    │
│  编译频率: 高 (硬件驱动迭代)                                          │
│                                                                       │
├───────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Layer 1: Core Layer (核心基础层)                                    │
│  ═══════════════════════════                                         │
│  • interface         - ROS2 消息/服务/动作定义                        │
│    ├─ msg/qr/       - 四足机器人消息                                  │
│    ├─ msg/arm/      - 机械臂消息                                      │
│    └─ srv/          - 服务定义                                        │
│  • system_bringup    - 分层启动系统                                  │
│    ├─ 1_hardware.launch.py      - 硬件层启动                         │
│    ├─ 2_perception.launch.py    - 感知层启动                         │
│    ├─ 3_intelligence.launch.py  - 智能层启动                         │
│    └─ 4_application.launch.py   - 应用层启动                         │
│  • third_party       - 第三方库 (backward_ros)                       │
│  依赖: ROS2 Humble 基础环境                                          │
│  编译频率: 极低 (消息协议变更)                                        │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### 2.2 ROS2 Overlay 机制

每一层通过 ROS2 的 overlay 机制叠加构建:

```bash
# Core Layer (Layer 1)
source /opt/ros/humble/setup.bash
cd src/core_layer && colcon build
source install/setup.bash

# Hardware Layer (Layer 2) 叠加 Core Layer
cd src/hardware_layer
source setup.bash  # 自动包含 core_layer
colcon build
source install/setup.bash

# Perception Layer (Layer 3) 叠加 Hardware + Core
cd src/perception_layer
source setup.bash  # 自动包含 hardware_layer + core_layer
colcon build
source install/setup.bash

# Intelligence Layer (Layer 4) 叠加 Perception + Hardware + Core
cd src/intelligence_layer
source setup.bash
colcon build
source install/setup.bash

# Application Layer (Layer 5) 叠加所有下层
cd src/application_layer
source setup.bash  # 包含全部 4 层
colcon build
source install/setup.bash
```

**好处**:
- 增量编译: 只重新编译修改的层
- 依赖清晰: 严格的单向依赖关系
- 平台适配: 不同硬件平台只需编译到对应层
  - 树莓派: 编译到 Layer 3 (Intelligence)
  - Jetson Orin: 编译到 Layer 4/5 (Application)

### 2.3 关键包说明

| 包名 | 层级 | 功能 | 关键特性 |
|------|------|------|---------|
| **interface** | Core | 消息定义 | 统一的数据协议 |
| **system_bringup** | Core | 启动系统 | 分层启动,平台适配 |
| **qr_control** | Hardware | 四足控制 | ros2_control,1000Hz |
| **robot_base** | Hardware | 底盘驱动 | 差速/全向轮支持 |
| **camera_ptz** | Perception | 云台相机 | 自动跟踪,目标检测 |
| **navigation** | Intelligence | 自主导航 | SLAM,路径规划 |
| **bt_manager** | Intelligence | 行为树 | 任务调度,状态机 |
| **dev_server** | Application | Web 服务 | HTTP API,WebSocket |
| **update_manager** | Application | OTA 升级 | 无缝更新,自动回滚 |

### 2.4 vcstool 多仓库管理

使用 `.repos` 文件管理 5 个分层仓库:

```yaml
repositories:
  core_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-core-layer.git
    version: main
  hardware_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-hardware-layer.git
    version: main
  perception_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-perception-layer.git
    version: main
  intelligence_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-intelligence-layer.git
    version: main
  application_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-application-layer.git
    version: main
```

**管理命令**:
```bash
# 导入所有仓库
./sync.sh import

# 检查状态
./sync.sh status

# 更新所有仓库
./sync.sh pull

# 推送所有改动
./sync.sh push
```

---

## 3. qr_wl → ROS2 Control 迁移架构

### 3.1 迁移背景

**当前 qr_wl 架构问题**:
- 单体应用,紧耦合 (~15K 行代码)
- 只支持实机,无仿真环境
- 500Hz 控制频率,延迟 8ms
- 内存泄漏,长时间运行不稳定
- 难以扩展新算法 (RL/MPC)

**ros2_control 架构优势**:
- 标准化接口,解耦硬件和控制逻辑
- 1000Hz 控制频率,延迟 3ms
- 支持 Gazebo/MuJoCo 仿真
- 无缝集成 ROS2 生态
- 开发效率提升 228%

### 3.2 迁移架构对比

```
┌─────────────────────────────────────────────────────────────────────┐
│                    qr_wl  vs  ros2_control 架构对比                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  qr_wl (传统单体架构)              ros2_control (目标架构)           │
│  ═══════════════════               ════════════════════             │
│                                                                       │
│  ┌──────────────────┐              ┌────────────────────────┐       │
│  │  qr_wl 可执行文件 │              │  ROS2 Nodes 生态系统    │       │
│  │  ══════════════  │              ├────────────────────────┤       │
│  │                  │              │  Application Layer     │       │
│  │  ├─ baseline/    │              │  ├─ dev_server         │       │
│  │  │  消息传递      │              │  ├─ remote_ctrl        │       │
│  │  │              │              │  └─ record             │       │
│  │  ├─ control/     │              ├────────────────────────┤       │
│  │  │  ├─ FSM       │              │  qr_sdk_bridge         │       │
│  │  │  └─ 控制算法   │              │  (兼容层)               │       │
│  │  │              │              ├────────────────────────┤       │
│  │  ├─ app/         │              │  Hardware Layer        │       │
│  │  │  ├─ motion/   │              │  ┌──────────────────┐ │       │
│  │  │  ├─ driver/   │              │  │  qr_control      │ │       │
│  │  │  ├─ network/  │              │  │  ══════════════  │ │       │
│  │  │  └─ robot/    │              │  │                  │ │       │
│  │  │              │              │  │  hardware/       │ │       │
│  │  └─ project/     │              │  │  ├─ QrSpiCan    │ │       │
│  │     主入口        │              │  │  │   Hardware   │ │       │
│  │                  │              │  │  │              │ │       │
│  └──────────────────┘              │  │  controller/     │ │       │
│                                     │  │  ├─ QrFsm       │ │       │
│  特点:                              │  │  │   Controller │ │       │
│  • 15K 行 C++                       │  │  │              │ │       │
│  • 500Hz 控制                       │  │  fsm/          │ │       │
│  • 只支持实机                        │  │  ├─ StateManager│ │       │
│  • 难以测试                          │  │  ├─ fsmStand  │ │       │
│  • SDK 紧耦合                        │  │  ├─ fsmWalk   │ │       │
│                                     │  │  └─ fsmLie     │ │       │
│                                     │  │                  │ │       │
│                                     │  └──────────────────┘ │       │
│                                     │                        │       │
│                                     │  sensor/               │       │
│                                     │  ├─ imu_node           │       │
│                                     │  └─ gamepad_node       │       │
│                                     └────────────────────────┘       │
│                                                                       │
│                                     特点:                             │
│                                     • 8K 行 C++ (减少 47%)            │
│                                     • 1000Hz 控制 (提升 100%)         │
│                                     • 支持 3 种仿真环境               │
│                                     • 标准化接口,易测试                │
│                                     • 解耦 SDK (可替换)               │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### 3.3 关键架构决策

#### 决策 1: FSM 位置 - 嵌入 Controller

**方案**: FSM 直接嵌入 `QrFsmController` 内部,不作为独立 ROS2 节点。

**理由**:
- 零延迟: FSM 状态计算和电机命令在同一个 1000Hz 循环
- 实时安全: 避免节点间通信延迟
- 代码复用: qr_wl 的 StateManager 可以直接迁移

**实现**:
```cpp
class QrFsmController : public controller_interface::ControllerInterface {
private:
    // 嵌入 qr_wl 的完整 FSM
    std::unique_ptr<StateManager> state_manager_;

    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override {

        // 1. 读取高层命令 (来自 remote_ctrl 话题)
        auto cmd = rt_command_ptr_.readFromRT();

        // 2. 以 500Hz 运行 FSM (从 1000Hz 抽取)
        if (++counter_ % 2 == 0) {
            state_manager_->Run();
        }

        // 3. 使用 FSM 目标执行 PD 控制
        auto targets = state_manager_->GetJointTargets();
        for (size_t i = 0; i < 12; ++i) {
            double torque = PdControl(targets[i], current_state[i]);
            command_interfaces_[i].set_value(torque);
        }

        return controller_interface::return_type::OK;
    }
};
```

#### 决策 2: 单一 `qr_control` 包

**方案**: 硬件接口 + 控制器 + FSM 放在一个包 `qr_control`。

**理由**:
- 简化依赖管理
- 四足特定代码集中维护
- SPI-CAN + MIT 电机是专用硬件,不需要拆分

**包结构**:
```
qr_control/
├── hardware/
│   ├── QrSpiCanHardware.hpp/cpp  # 硬件接口
│   └── spi_can_driver.cpp        # SPI-CAN 驱动
├── controller/
│   └── QrFsmController.hpp/cpp   # 控制器
├── fsm/
│   ├── StateManager.cpp           # qr_wl 状态管理器
│   ├── fsmStand.cpp               # 站立状态
│   ├── fsmWalk.cpp                # 行走状态
│   └── fsmLie.cpp                 # 趴下状态
├── urdf/
│   └── qr_robot.urdf.xacro        # 机器人描述
└── config/
    ├── qr_controllers.yaml        # 控制器配置
    └── qr_hardware.yaml           # 硬件参数
```

### 3.4 迁移收益分析

基于 `ros2_control/EXECUTIVE_SUMMARY_CN.md` 的详细分析:

| 指标 | qr_wl | ros2_control | 改进 |
|------|-------|--------------|------|
| **控制频率** | 500 Hz | 1000 Hz | ⬆️ +100% |
| **命令延迟** | 8 ms | 3 ms | ⬇️ -62.5% |
| **代码量** | 15K 行 | 8K 行 | ⬇️ -47% |
| **连续运行** | 4 小时 | 24+ 小时 | ⬆️ +500% |
| **支持平台** | 仅实机 | 实机 + 3 种仿真 | ⬆️ +300% |
| **开发效率** | 基准 | 提升 328% | ⬆️ +228% |

**投资回报率**:
- **一次性投入**: 11.5 天 (详细迁移计划)
- **年化收益**: 160 人日 (开发效率提升)
- **回本周期**: 26 天 (约 1 个月)
- **第一年 ROI**: 1291%

### 3.5 迁移时间线

```
═══════════════════════════════════════════════════════════════════
Week 1:  硬件接口开发 (QrSpiCanHardware)       ⏰ 关键路径
Week 2:  控制器 + FSM 迁移 (QrFsmController)   ⏰ 关键路径
Week 3:  传感器集成 + 配置 (IMU/Gamepad)       ⏰ 重要
Week 4:  测试验证 + 上线 (实机 + 仿真)          ⏰ 里程碑
═══════════════════════════════════════════════════════════════════
总计: 11.5 工作日 (~2.3 周)
═══════════════════════════════════════════════════════════════════
```

详细迁移计划参考: `/home/wl/twh/workspace/ros2_control/QR_WL_MIGRATION_PLAN.md`

---

## 4. qr_chart 监控客户端

### 4.1 架构概览

qr_chart 是基于 Qt6 的跨平台监控和数据可视化工具,支持实时数据显示和历史数据分析。

```
┌─────────────────────────────────────────────────────────────────────┐
│                      qr_chart 客户端架构                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │                    UI Layer (Qt6 Widgets)                  │      │
│  │  ═════════════════════════════════════════                │      │
│  │                                                             │      │
│  │  MainWindow                                                │      │
│  │  ├─ Tab: Quadruped (chartQr)   - 四足机器人可视化          │      │
│  │  ├─ Tab: Arm (chartArm)        - 机械臂可视化              │      │
│  │  ├─ Tab: Human (chartHuman)    - 人形机器人可视化          │      │
│  │  ├─ uiReadDatabase             - 历史数据查看              │      │
│  │  ├─ uiWriteDatabase            - 数据记录                  │      │
│  │  └─ uiSearchRobot              - 机器人搜索                │      │
│  │                                                             │      │
│  └─────────────────────────────────────────────────────────────┘      │
│                              │                                         │
│                              ▼                                         │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │          Visualization Layer (QCustomPlot)                 │      │
│  │  ═══════════════════════════════════════                  │      │
│  │                                                             │      │
│  │  chartWidget                                               │      │
│  │  ├─ QCustomPlot Graphs    - 实时曲线绘制                   │      │
│  │  ├─ chartTabData          - 分页数据管理                   │      │
│  │  └─ chartQr/Arm/Human     - 机器人类型特化                 │      │
│  │                                                             │      │
│  └─────────────────────────────────────────────────────────────┘      │
│                              │                                         │
│                              ▼                                         │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │             Data Management Layer                          │      │
│  │  ═══════════════════════════                              │      │
│  │                                                             │      │
│  │  DataManager (Global Singleton)                           │      │
│  │  ├─ SharedMemorySpace      - 共享数据缓冲区                │      │
│  │  ├─ LegInfoSet             - 四足腿部数据                  │      │
│  │  ├─ ImuInfoSet             - IMU 传感器数据                │      │
│  │  ├─ ArmInfoSet             - 机械臂关节数据                │      │
│  │  ├─ HumanMotorSet          - 人形电机数据                  │      │
│  │  └─ WatchInfoSet           - 通用监控数据                  │      │
│  │                                                             │      │
│  │  Database (SQLite)                                         │      │
│  │  ├─ Session Management     - 会话存储                      │      │
│  │  ├─ Time-series Data       - 时序数据                      │      │
│  │  └─ dbVersion/             - 版本化 Schema                 │      │
│  │                                                             │      │
│  └─────────────────────────────────────────────────────────────┘      │
│                              │                                         │
│                              ▼                                         │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │             Network Layer (IIRI SDK)                       │      │
│  │  ═══════════════════════════════                          │      │
│  │                                                             │      │
│  │  DebugClient (TCP Client)                                 │      │
│  │  ├─ Port: 20333 (default)  - 机器人数据端口                │      │
│  │  ├─ Interval: 2ms (500Hz)  - 实时数据流                    │      │
│  │  └─ Protocol: IIRI Binary  - 自定义二进制协议              │      │
│  │                                                             │      │
│  │  debugWatch (SDK Protocol Handler)                        │      │
│  │  ├─ Message Parsing        - 协议解析                      │      │
│  │  ├─ Callback System        - 数据回调                      │      │
│  │  └─ Connection Management  - 连接管理                      │      │
│  │                                                             │      │
│  └─────────────────────────────────────────────────────────────┘      │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

### 4.2 关键数据结构

```cpp
// DataCollector/DataStruct.h

// 四足腿部信息
struct LegInfoSet {
    float q[12];          // 关节角度
    float dq[12];         // 关节速度
    float tau[12];        // 关节力矩
    float q_des[12];      // 目标角度
    float dq_des[12];     // 目标速度
    float tau_des[12];    // 目标力矩
};

// IMU 传感器信息
struct ImuInfoSet {
    float quaternion[4];   // 四元数
    float gyroscope[3];    // 角速度
    float accelerometer[3];// 加速度
    float rpy[3];          // 欧拉角
};

// 机械臂关节信息
struct ArmInfoSet {
    int joint_count;
    float position[16];
    float velocity[16];
    float effort[16];
};

// 人形机器人电机信息
struct HumanMotorSet {
    int motor_count;
    float angle[32];
    float torque[32];
    float temperature[32];
};
```

### 4.3 实时数据流

```
机器人设备 (192.168.1.54)                qr_chart 客户端 (PC)
═══════════════════════                 ════════════════════

┌─────────────────┐                      ┌─────────────────┐
│  dev_server     │                      │  DebugClient    │
│  (ROS2 Bridge)  │                      │  (TCP Client)   │
└─────────────────┘                      └─────────────────┘
        │                                        │
        │  TCP 连接建立                           │
        │◄───────────────────────────────────────┤
        │                                        │
        │  每 2ms 发送数据包                      │
        ├────────────────────────────────────────►│
        │  [Header|LegInfo|ImuInfo|...]          │
        │                                        │
        │                                        ▼
        │                              ┌─────────────────┐
        │                              │  debugWatch     │
        │                              │  (协议解析)      │
        │                              └─────────────────┘
        │                                        │
        │                                        ▼
        │                              ┌─────────────────┐
        │                              │  DataManager    │
        │                              │  (数据更新)      │
        │                              └─────────────────┘
        │                                        │
        │                                        ▼
        │                              ┌─────────────────┐
        │                              │  chartWidget    │
        │                              │  (UI 刷新)       │
        │                              └─────────────────┘
```

### 4.4 多机器人类型支持

qr_chart 同时支持三种机器人类型的监控:

| 机器人类型 | 可视化组件 | 主要数据 | 特殊功能 |
|-----------|-----------|---------|---------|
| **四足机器人** | chartQr | LegInfoSet, ImuInfoSet | 步态分析,腿部轨迹 |
| **机械臂** | chartArm | ArmInfoSet, ArmTeachSet | 轨迹跟踪,示教编程 |
| **人形机器人** | chartHuman | HumanMotorSet | 全身运动协调 |

### 4.5 构建和部署

```bash
# 依赖安装 (Ubuntu 20.04+)
sudo apt install qt6-base-dev libqt6charts6-dev libboost-all-dev

# 构建
mkdir build && cd build
cmake ..
make -j$(nproc)

# 运行
./qrChart
```

**打包部署**:
- 便携式包: `./package.sh` → `qrChart-v1.0.0-linux-x86_64.tar.gz`
- 系统安装: `sudo ./install.sh`
- AppImage: `./create_appimage.sh`

---

## 5. OTA 更新系统

### 5.1 update_manager 架构

OTA (Over-The-Air) 更新系统提供远程无缝升级能力,支持自动备份和回滚。

```
┌─────────────────────────────────────────────────────────────────────┐
│                    update_manager OTA 系统架构                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │                 Main Orchestrator                          │      │
│  │  ════════════════════════                                 │      │
│  │                                                             │      │
│  │  main.cpp                                                  │      │
│  │  ├─ 参数解析: --task-id, --package, --app-type            │      │
│  │  ├─ 配置加载: UpdateConfig                                │      │
│  │  └─ 流程编排: 调度各个管理器                               │      │
│  │                                                             │      │
│  └─────────────────────────────────────────────────────────────┘      │
│                              │                                         │
│                              ▼                                         │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │              Core Managers (6 个管理器)                     │      │
│  │  ════════════════════════════════════                     │      │
│  │                                                             │      │
│  │  1. StatusManager           - 状态管理和日志               │      │
│  │     ├─ setState()           - 更新状态机                   │      │
│  │     ├─ setProgress()        - 进度跟踪 (0-100%)            │      │
│  │     ├─ log()                - 写入日志文件                 │      │
│  │     └─ saveStatus()         - JSON 状态持久化              │      │
│  │                                                             │      │
│  │  2. FileVerifier            - 文件完整性校验               │      │
│  │     ├─ calculateSHA256()    - 计算文件哈希                 │      │
│  │     ├─ verifySHA256()       - 校验哈希值                   │      │
│  │     └─ verifyGPGSignature() - GPG 签名验证 (可选)          │      │
│  │                                                             │      │
│  │  3. BackupManager           - 备份和回滚                   │      │
│  │     ├─ createBackup()       - 创建完整备份                 │      │
│  │     ├─ verifyBackup()       - 验证备份完整性               │      │
│  │     ├─ rollback()           - 回滚到备份版本               │      │
│  │     └─ cleanup()            - 清理旧备份                   │      │
│  │                                                             │      │
│  │  4. SystemdManager          - 系统服务管理                 │      │
│  │     ├─ stopService()        - 停止 ROS2 服务               │      │
│  │     ├─ startService()       - 启动 ROS2 服务               │      │
│  │     ├─ restartService()     - 重启服务                     │      │
│  │     └─ getServiceStatus()   - 查询服务状态                 │      │
│  │                                                             │      │
│  │  5. HealthChecker           - 健康检查                     │      │
│  │     ├─ checkRosTopics()     - 检查 ROS2 话题               │      │
│  │     ├─ checkServices()      - 检查 ROS2 服务               │      │
│  │     ├─ checkProcesses()     - 检查关键进程                 │      │
│  │     └─ performHealthCheck() - 综合健康评估                 │      │
│  │                                                             │      │
│  │  6. CommandExecutor         - 命令执行器                   │      │
│  │     ├─ execute()            - 执行 shell 命令              │      │
│  │     ├─ executeWithTimeout() - 带超时执行                   │      │
│  │     └─ captureOutput()      - 捕获输出和错误               │      │
│  │                                                             │      │
│  └─────────────────────────────────────────────────────────────┘      │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

### 5.2 更新流程状态机

```
┌─────────────────────────────────────────────────────────────────────┐
│                      OTA 更新流程状态机                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  [CREATED]           初始状态: 任务创建                               │
│      │                                                                │
│      ▼                                                                │
│  [PREPARING]         准备阶段                                         │
│      │               ├─ 解压更新包                                    │
│      │               ├─ SHA256 校验                                   │
│      │               └─ GPG 签名验证 (可选)                           │
│      ▼                                                                │
│  [RUNNING]           执行阶段                                         │
│      │               ├─ 停止 systemd 服务 (iiri-ros.service)          │
│      │               ├─ 创建完整备份                                  │
│      │               ├─ 复制新文件到目标位置                          │
│      │               ├─ 更新 symlink (/home/wl/autorun/iiri-ros)     │
│      │               ├─ 启动服务                                      │
│      │               └─ 健康检查 (30 秒超时)                          │
│      │                                                                │
│      ├──► [SUCCESS]   更新成功                                        │
│      │                 ├─ 清理临时文件                                │
│      │                 └─ 删除旧备份 (保留最近 3 个)                  │
│      │                                                                │
│      └──► [FAILED]     更新失败                                       │
│             │          ├─ 记录错误信息                                │
│             ▼          └─ 触发回滚                                    │
│         [ROLLBACK]     回滚流程                                       │
│             │          ├─ 停止当前服务                                │
│             │          ├─ 恢复备份文件                                │
│             │          ├─ 恢复 symlink                                │
│             │          └─ 重启服务                                    │
│             │                                                         │
│             ├──► [ROLLBACK_SUCCESS]  回滚成功                         │
│             │                                                         │
│             └──► [ROLLBACK_FAILED]   回滚失败 (需人工介入)            │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

### 5.3 关键数据结构

```cpp
// update_types.hpp

enum class UpdateState {
    CREATED,          // 初始状态
    PREPARING,        // 准备中
    RUNNING,          // 执行中
    SUCCESS,          // 成功
    FAILED,           // 失败
    ROLLBACK,         // 回滚中
    ROLLBACK_SUCCESS, // 回滚成功
    ROLLBACK_FAILED,  // 回滚失败
    CANCELLED         // 取消
};

struct UpdateConfig {
    int maxRetries = 3;
    int healthCheckTimeout = 30;       // 秒
    int healthCheckRetries = 10;
    int commandTimeout = 300;          // 秒
    bool enableRollback = true;
    bool strictMode = false;
    std::string logLevel = "INFO";
    std::string logDir = "/var/log/update_manager";
    std::string backupDir = "/var/backups/iiri";
    std::string statusFile = "/var/run/update_status.json";
};

struct UpdateTask {
    std::string taskId;
    std::string packagePath;
    std::string appType;           // "ros2" or "qr"
    UpdateState state;
    int progress;                  // 0-100
    std::string message;
    std::string errorMsg;
    std::chrono::system_clock::time_point startTime;
    std::chrono::system_clock::time_point endTime;
};
```

### 5.4 部署架构

```
机器人设备 (192.168.1.54)
═══════════════════════════

┌───────────────────────────────────────────────────────┐
│  /home/wl/autorun/                                    │
│  ├─ iiri-ros-arm-255170c/      ← 当前版本 (symlink)   │
│  ├─ iiri-ros-arm-c118d41/      ← 上一版本             │
│  ├─ iiri-ros-arm-258eae4/      ← 更早版本             │
│  ├─ iiri-ros → iiri-ros-arm-255170c/  (symlink)      │
│  └─ update_manager/                                   │
│      ├─ update_manager          ← OTA 程序            │
│      ├─ deploy_update_manager.sh                      │
│      └─ run_remote_test.sh                            │
└───────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────┐
│  /var/backups/iiri/                                   │
│  ├─ iiri-ros-backup-1730275200/  ← 最新备份           │
│  ├─ iiri-ros-backup-1730189800/  ← 备份-1             │
│  └─ iiri-ros-backup-1730103400/  ← 备份-2             │
└───────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────┐
│  /var/log/update_manager/                             │
│  ├─ update_task-1730275200.log                        │
│  ├─ update_task-1730189800.log                        │
│  └─ update_task-1730103400.log                        │
└───────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────┐
│  systemd 服务                                          │
│  /etc/systemd/system/iiri-ros.service                 │
│  ExecStart=/home/wl/autorun/iiri-ros/start_ros2.sh    │
│  (通过 symlink 自动指向最新版本)                       │
└───────────────────────────────────────────────────────┘
```

### 5.5 OTA 升级命令

```bash
# 手动 OTA 升级
ssh wl@192.168.1.54
cd /home/wl/autorun/update_manager
./update_manager \
  --task-id "manual-$(date +%s)" \
  --package /tmp/iiri-ros-arm-<commit>.tar.gz \
  --app-type ros2

# 检查状态
cat /var/run/update_status.json

# 查看日志
tail -f /var/log/update_manager/update_*.log

# 手动回滚 (如果 OTA 失败)
cd /var/backups/iiri/iiri-ros-backup-<timestamp>
sudo systemctl stop iiri-ros.service
sudo cp -r * /home/wl/autorun/iiri-ros/
sudo systemctl start iiri-ros.service
```

### 5.6 安全特性

| 特性 | 实现 | 作用 |
|------|------|------|
| **完整性校验** | SHA256 哈希 | 防止包损坏或篡改 |
| **签名验证** | GPG 签名 (可选) | 防止恶意包 |
| **原子更新** | Symlink 切换 | 要么成功要么不变 |
| **自动备份** | 完整目录复制 | 快速回滚 |
| **健康检查** | ROS2 话题/服务检测 | 确保功能正常 |
| **超时保护** | 30 秒健康检查超时 | 防止挂起 |
| **日志审计** | 详细日志记录 | 问题追溯 |

---

## 6. 网络通信架构

### 6.1 通信协议栈

```
┌─────────────────────────────────────────────────────────────────────┐
│                       网络通信协议栈                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  qr_chart (PC)              dev_server (机器人)        ROS2 Nodes    │
│  ═══════════                ══════════════            ═══════════    │
│                                                                       │
│  ┌──────────────┐           ┌──────────────┐         ┌───────────┐  │
│  │              │  TCP      │   IIRI SDK   │  ROS2   │           │  │
│  │  DebugClient ├──────────►│   Bridge     ├────────►│  Topics   │  │
│  │              │  20333    │              │  (DDS)  │           │  │
│  └──────────────┘           └──────────────┘         └───────────┘  │
│       │                             │                      │         │
│       │                             │                      │         │
│       ▼                             ▼                      ▼         │
│  二进制协议                    话题订阅/发布              DDS 通信    │
│  ├─ Header                    ├─ /joint_states           ├─ 本地     │
│  ├─ LegInfoSet                ├─ /imu_data               ├─ 远程     │
│  ├─ ImuInfoSet                ├─ /cmd_vel                └─ 组播     │
│  ├─ ArmInfoSet                ├─ /camera_image                       │
│  └─ Timestamp                 └─ /rosout                             │
│                                                                       │
│  ┌──────────────┐           ┌──────────────┐                        │
│  │              │  WS       │   HTTP API   │                        │
│  │  Web Browser ├──────────►│   (CrowCpp)  │                        │
│  │  (Vue3)      │  8080     │              │                        │
│  └──────────────┘           └──────────────┘                        │
│       │                             │                                │
│       │                             │                                │
│       ▼                             ▼                                │
│  WebSocket                     RESTful API                           │
│  ├─ 连接状态                    ├─ GET /api/robot/status             │
│  ├─ 实时控制                    ├─ POST /api/control/velocity        │
│  └─ 数据流                      ├─ GET /api/sensors/imu              │
│                                └─ POST /api/update/trigger           │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### 6.2 端口映射表

| 端口 | 协议 | 服务 | 用途 |
|------|------|------|------|
| **20333** | TCP | IIRI SDK 服务器 | qr_chart 实时数据流 |
| **8080** | HTTP/WS | dev_server | Web 界面和 API |
| **22** | SSH | SSH 服务 | OTA 升级,远程管理 |
| **11311** | TCP/UDP | ROS1 Master (legacy) | 兼容 qr_wl |
| **DDS** | UDP | ROS2 DDS | 节点间通信 |

### 6.3 数据流向

```
┌─────────────────────────────────────────────────────────────────────┐
│                    实时数据流向 (500Hz)                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  硬件层                感知层              智能层           应用层    │
│  ═════                ═════              ═════           ═════      │
│                                                                       │
│  motion_control       camera_ptz        navigation      dev_server  │
│  robot_base           tts               bt_manager      remote_ctrl │
│  sensor/imu           speaker           smart_follow    record      │
│      │                    │                  │               │       │
│      │                    │                  │               │       │
│      └────────┬───────────┴──────────────────┴───────────────┘       │
│               │                                                       │
│               ▼                                                       │
│         ROS2 DDS Topics                                              │
│         ═══════════════                                              │
│         /joint_states       - 关节状态 (12 个关节 × 500Hz)           │
│         /imu_data           - IMU 数据 (500Hz)                       │
│         /cmd_vel            - 速度命令 (50Hz)                        │
│         /camera_image       - 图像流 (30Hz)                          │
│         /tts_output         - 语音输出 (事件触发)                     │
│         /bt_status          - 行为树状态 (10Hz)                      │
│               │                                                       │
│               ├──────────┬────────────────────────────┐              │
│               ▼          ▼                            ▼              │
│         dev_server   qr_chart                   rviz2/foxglove       │
│         (ROS2桥接)   (TCP客户端)                (可视化)              │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### 6.4 远程控制流

```
用户输入 → Web/qr_chart → dev_server → ROS2 Topics → 机器人执行
═══════   ═════════════   ══════════   ═══════════   ══════════

1. 用户操作:
   ├─ Web 界面按钮点击
   └─ qr_chart 参数调整

2. 命令发送:
   ├─ WebSocket 消息 (JSON)
   └─ TCP 数据包 (二进制)

3. dev_server 处理:
   ├─ 解析命令
   ├─ 验证安全性
   └─ 发布 ROS2 话题

4. 机器人响应:
   ├─ remote_ctrl 订阅命令
   ├─ bt_manager 任务调度
   └─ qr_control 执行运动

5. 状态反馈:
   ├─ 传感器数据上报
   ├─ 状态机更新
   └─ UI 实时刷新
```

---

## 7. 部署和运维架构

### 7.1 CI/CD 流水线

```
┌─────────────────────────────────────────────────────────────────────┐
│                      Jenkins CI/CD 流水线                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  1. 触发器                                                            │
│     ├─ 定时触发: 每 2 小时 (H */2 * * *)                             │
│     ├─ Git Push: 主分支提交                                          │
│     └─ 手动触发: Jenkins UI                                          │
│                                                                       │
│  2. 代码导入                                                          │
│     ├─ vcs import src < .repos                                       │
│     ├─ 下载所有 5 层仓库                                              │
│     └─ 生成 Git 版本信息                                              │
│                                                                       │
│  3. Docker 编译                                                       │
│     ├─ 镜像: 192.168.1.93/iiri/build_x86_ros2:latest                 │
│     ├─ 分层编译: core → hardware → ... → application                 │
│     ├─ 并行度: 单任务 (避免内存不足)                                  │
│     └─ 输出: build_x86_shared/install/                               │
│                                                                       │
│  4. 构建产物归档                                                      │
│     ├─ 压缩: iiri-ros-x86-<commit>.tar.gz                            │
│     ├─ SHA256: 生成校验文件                                          │
│     └─ 存储: /var/jenkins_home/workspace/artifacts/                 │
│                                                                       │
│  5. 测试 (可选)                                                       │
│     ├─ 单元测试: colcon test                                         │
│     ├─ 集成测试: system_bringup 启动测试                             │
│     └─ 性能测试: ROS2 topic 频率检测                                 │
│                                                                       │
│  6. 部署                                                              │
│     ├─ 上传到文件服务器: scp → 192.168.1.54:/tmp/                    │
│     ├─ 触发 OTA 升级: SSH 调用 update_manager                        │
│     └─ 验证部署: 健康检查                                            │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### 7.2 Docker 镜像管理

```
Harbor 镜像仓库 (192.168.1.93)
════════════════════════════

┌──────────────────────────────────────────────────────┐
│  iiri/build_x86_ros2                                 │
│  ├─ latest           ← 当前开发版                    │
│  ├─ v1.4.3           ← 稳定版本                      │
│  └─ v1.4.2           ← 上一版本                      │
│                                                       │
│  包含:                                                │
│  ├─ ROS2 Humble 完整环境                             │
│  ├─ vcstool, colcon 构建工具                         │
│  ├─ 编译依赖 (OpenCV, PCL, Eigen)                    │
│  └─ CUDA 支持 (GPU 版本)                             │
└──────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────┐
│  iiri/build_arm_ros2                                 │
│  ├─ latest           ← 当前开发版                    │
│  ├─ v1.4.2           ← 稳定版本                      │
│  └─ v1.4.1           ← 上一版本                      │
│                                                       │
│  包含:                                                │
│  ├─ ROS2 Humble ARM64 环境                           │
│  ├─ Jetson 优化编译标志                              │
│  ├─ NVIDIA runtime 支持                              │
│  └─ 交叉编译工具链                                   │
└──────────────────────────────────────────────────────┘
```

### 7.3 Systemd 服务管理

```
机器人设备端 systemd 服务
══════════════════════════

┌───────────────────────────────────────────────────────┐
│  /etc/systemd/system/iiri-ros.service                 │
│  ═══════════════════════════════════                  │
│                                                        │
│  [Unit]                                               │
│  Description=IIRI ROS2 Robot System                   │
│  After=network.target                                 │
│                                                        │
│  [Service]                                            │
│  Type=simple                                          │
│  User=wl                                              │
│  WorkingDirectory=/home/wl/autorun/iiri-ros           │
│  ExecStart=/home/wl/autorun/iiri-ros/start_ros2.sh    │
│  Restart=on-failure                                   │
│  RestartSec=10                                        │
│  Environment="ROS_LOG_DIR=/tmp/ros2_logs"             │
│  Environment="RCUTILS_LOGGING_USE_STDOUT=0"           │
│                                                        │
│  [Install]                                            │
│  WantedBy=multi-user.target                           │
└───────────────────────────────────────────────────────┘

操作命令:
═══════

# 启动服务
sudo systemctl start iiri-ros.service

# 查看状态
sudo systemctl status iiri-ros.service

# 查看日志
journalctl -u iiri-ros.service -f

# 开机自启
sudo systemctl enable iiri-ros.service

# OTA 升级后重启
sudo systemctl restart iiri-ros.service
```

### 7.4 版本管理策略

```
Git 仓库版本管理
══════════════

主仓库: iiri_ros2_architecture (master)
├─ 提交: c118d41
├─ 标签: v1.0.0, v1.1.0
└─ 子模块指针:
    ├─ core_layer @ 28122ad
    ├─ hardware_layer @ 258eae4
    ├─ perception_layer @ 522a395
    ├─ intelligence_layer @ 18521f1
    └─ application_layer @ 1f3a6a4

发布版本命名:
iiri-ros-<arch>-<commit>[-<tag>].tar.gz
├─ iiri-ros-arm-c118d41.tar.gz      (开发版)
├─ iiri-ros-arm-c118d41-v1.0.0.tar.gz  (发布版)
└─ iiri-ros-arm-c118d41-v1.0.0.tar.gz.sha256  (校验)

部署版本回退:
/home/wl/autorun/
├─ iiri-ros → iiri-ros-arm-c118d41/  (当前)
├─ iiri-ros-arm-c118d41/             (版本 3)
├─ iiri-ros-arm-258eae4/             (版本 2)
└─ iiri-ros-arm-24c7b89/             (版本 1)
```

---

## 8. 未来演进路线图

### 8.1 短期目标 (3-6 个月)

```
Q1 2025: qr_wl → ros2_control 迁移
════════════════════════════════

✅ 阶段 1: 硬件接口开发           (2 天)
✅ 阶段 2: 控制器 + FSM 迁移      (3 天)
⏳ 阶段 3: 传感器集成             (2 天)
⏳ 阶段 4: 实机测试               (2 天)
⏳ 阶段 5: Gazebo 仿真集成        (1.5 天)
⏳ 阶段 6: MuJoCo 仿真集成        (1 天)

预期收益:
- 控制频率: 500Hz → 1000Hz
- 开发效率: 提升 228%
- ROI: 第一年 1291%
```

### 8.2 中期目标 (6-12 个月)

```
Q2-Q3 2025: 高级功能扩展
═══════════════════════

1. 强化学习集成
   ├─ MuJoCo 环境搭建
   ├─ PPO/SAC 算法实现
   └─ 实机迁移学习

2. 视觉导航增强
   ├─ Visual SLAM 集成
   ├─ 目标检测和跟踪
   └─ 语义地图构建

3. 多机器人协同
   ├─ 编队导航
   ├─ 任务分配
   └─ 通信拓扑优化

4. 云端监控平台
   ├─ Web 监控界面完善
   ├─ 多机器人管理
   └─ 数据分析面板
```

### 8.3 长期愿景 (12+ 个月)

```
2026+: 平台化和商业化
═══════════════════

1. 模块化机器人平台
   ├─ 支持更多机器人类型
   ├─ 硬件抽象层标准化
   └─ 插件式算法集成

2. 边缘计算优化
   ├─ 轻量化部署
   ├─ 模型量化和加速
   └─ 实时性能优化

3. 安全和认证
   ├─ 加密通信
   ├─ 访问控制
   └─ 审计日志

4. 商业化支持
   ├─ 客户定制化工具
   ├─ 技术支持平台
   └─ 培训和文档
```

---

## 9. 附录

### 9.1 关键性能指标 (KPI)

| 指标 | 当前值 | 目标值 | 备注 |
|------|-------|-------|------|
| **控制频率** | 500 Hz | 1000 Hz | qr_control 迁移后 |
| **命令延迟** | 8 ms | 3 ms | 端到端延迟 |
| **系统稳定性** | 4 小时 | 24+ 小时 | 无故障运行时间 |
| **OTA 升级时间** | N/A | < 5 分钟 | 含健康检查 |
| **代码覆盖率** | ~30% | 80% | 单元测试 + 集成测试 |

### 9.2 技术栈总览

| 层级 | 技术 | 版本 |
|------|------|------|
| **操作系统** | Ubuntu | 22.04 LTS |
| **ROS** | ROS2 Humble | Latest |
| **构建系统** | colcon | 0.12+ |
| **版本管理** | vcstool | 0.3.0 |
| **CI/CD** | Jenkins | 2.x |
| **容器** | Docker | 20.10+ |
| **镜像仓库** | Harbor | 2.x |
| **监控客户端** | Qt6 | 6.2+ |
| **可视化** | QCustomPlot | 2.x |
| **数据库** | SQLite | 3.x |

### 9.3 参考文档

- [iiri_ros2_architecture README](/home/wl/twh/workspace/iiri_ros2_architecture/README.md)
- [qr_wl 迁移计划](/home/wl/twh/workspace/ros2_control/QR_WL_MIGRATION_PLAN.md)
- [qr_wl 执行摘要](/home/wl/twh/workspace/ros2_control/EXECUTIVE_SUMMARY_CN.md)
- [qr_chart README](/home/wl/twh/workspace/client/qr_chart/README.md)
- [update_manager 实现总结](/home/wl/twh/workspace/iiri_ros2_architecture/src/application_layer/dev_server/update_manager/IMPLEMENTATION_SUMMARY.md)
- [ROS2 Control 官方文档](https://control.ros.org/)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)

### 9.4 联系方式

**项目负责人**: 唐文浩
**团队**: 西湖大学智能产业研究院
**仓库**: http://192.168.1.55/ontology/iiri_ros2_architecture
**Jenkins**: http://192.168.1.59:8081
**Harbor**: http://192.168.1.93

---

**文档版本**: v1.0.0
**最后更新**: 2025-10-30
**维护者**: 唐文浩
