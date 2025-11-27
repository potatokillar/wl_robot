# IIRI Hardware Layer

硬件抽象层 - 提供硬件驱动和底层接口。

## 包含内容

- **motion_control**: 运动控制相关
- **robot_base**: 机器人基础硬件接口
- **sensor**: 传感器驱动
  - camera: 摄像头驱动 (USB摄像头, D435 RGB摄像头)
  - lidar: 激光雷达驱动 (Livox, RS雷达)

## 依赖关系

- 依赖: Core Layer (interface消息定义)
- 被依赖: Perception Layer

## 构建方法

```bash
# 确保core_layer已构建并sourced
source /path/to/core_layer/install/setup.bash
source setup.bash
colcon build
```

## 版本

- 当前版本: v1.0.0
- 兼容ROS2: Humble

---
📦 Part of IIRI ROS2 Architecture