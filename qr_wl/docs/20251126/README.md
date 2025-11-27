# qr_wl 机器人控制系统文档

**版本**: 1.0.0  
**日期**: 2025-11-26  
**状态**: 正式发布

## 1. 系统概述

`qr_wl` 是一个通用的机器人控制系统，支持四足机器人（Quadruped）和机械臂（Robotic Arm）等多种形态。系统采用模块化分层架构设计，实现了硬件抽象、核心控制逻辑与应用接口的解耦。

本系统主要包含以下核心功能：
- **多形态支持**: 同时支持轮腿式四足机器人和多自由度机械臂。
- **硬件抽象**: 通过统一的接口层屏蔽底层传感器和执行器的差异。
- **事件驱动**: 基于 `MiniServer` 的事件循环机制，实现高效的异步任务处理。
- **多模态交互**: 集成语音交互、音频播放和视觉反馈功能。

## 2. 文档索引

本文档体系包含以下部分：

### 2.1 架构与设计
- [系统架构设计 (System Architecture)](system-architecture.md)
  - 总体架构图
  - 模块交互关系
  - 核心设计理念

### 2.2 核心模块说明
- [核心控制模块 (Core Control Modules)](modules/module-core-control.md)
  - UserQuadNode / UserArmNode
  - RobotState (状态机)
  - GamepadNode (手柄控制)
- [硬件抽象层 (Hardware Abstraction Layer)](modules/module-hardware-abstraction.md)
  - CanBridge (CAN通信)
  - ImuDrv (IMU驱动)
  - DeviceCustomParam (参数管理)
- [交互与服务模块 (Interaction & Services)](modules/module-interaction.md)
  - MiniServer (事件中心)
  - RobotMedia & OfflineVoice (媒体与语音)

### 2.3 参考资料
- [数据字典 (Data Dictionary)](data-dictionary.md)
- [异常处理流程 (Exception Handling)](exception-handling.md)
- [文档编写规范 (Style Guide)](style-guide.md)
- [更新日志 (CHANGELOG)](CHANGELOG.md)

## 3. 快速开始

### 环境要求
- Linux (推荐 Ubuntu 20.04/22.04)
- C++17 编译器
- CMake 3.10+

### 编译与运行
```bash
# 1. 创建构建目录
mkdir build && cd build

# 2. 编译项目
cmake ..
make -j4

# 3. 运行主程序 (需指定配置文件)
./project/real/qr_real config/qr-gazebo.toml
```

## 4. 联系方式

如有疑问或建议，请联系开发团队或提交 Issue。
