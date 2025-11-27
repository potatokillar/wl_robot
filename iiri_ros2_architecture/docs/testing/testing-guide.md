# System Bringup 测试指南

> **文档位置**: `docs/testing/testing-guide.md`
> **最后更新**: 2025-10-13
> **作者**: 唐文浩

本文档说明如何使用 Docker 容器对 `system_bringup` 包进行分层测试。

## 📋 测试环境

- **Docker镜像**: `192.168.1.93/iiri/build_x86_ros2:v1.4.3`
- **测试包**: `system_bringup` (位于 core_layer)
- **测试脚本**: `test_system_bringup.sh`

## 🚀 快速测试

### 方法1: 自动化测试（推荐）

```bash
# 1. 确保core_layer已编译
./build_layered.sh core_layer

# 2. 启动Docker容器并运行测试
./docker.sh run

# 在容器内执行
./test_in_docker.sh
```

### 方法2: 手动测试

```bash
# 1. 启动Docker容器
./docker.sh run

# 2. 在容器内source环境
source /opt/ros/humble/setup.bash
source build_x86_shared/install/setup.bash

# 3. 运行完整测试
./test_system_bringup.sh
```

## 📝 测试内容

脚本会自动测试以下启动文件：

### 分层测试
1. **1_hardware.launch.py** - 硬件层测试
   - motion_control (motion_control_node)
   - robot_base (robot_base_node)

2. **2_perception.launch.py** - 感知层测试
   - camera_ptz
   - speaker
   - tts
   - speech_recognition

3. **3_intelligence.launch.py** - 智能层测试
   - bt_manager
   - xiaozhi
   - path_tracker
   - smart_follow

4. **4_application.launch.py** - 应用层测试
   - dev_server
   - remote_ctrl
   - record

### 平台测试
5. **qr_debug.launch.py** - 调试平台（仅硬件层）
6. **qr_raspi.launch.py** - 树莓派平台（3层）
7. **qr_orin.launch.py** - Orin平台（4层）
8. **qr_arm.launch.py** - ARM平台（3层）

## 📊 测试结果

测试完成后会生成以下文件：

```
test_logs_<timestamp>/
├── test_report.md              # Markdown格式测试报告
├── 1_hardware.log              # 硬件层日志
├── 1_hardware_nodes.txt        # 硬件层节点列表
├── 1_hardware_topics.txt       # 硬件层话题列表
├── 1_hardware_result.txt       # 硬件层测试结果
├── 2_perception.log            # 感知层日志
├── ... (其他层级类似)
└── qr_*.log/nodes/topics/result.txt  # 平台测试结果
```

### 查看测试报告

```bash
# 查看最新的测试报告
cat test_logs_*/test_report.md

# 或直接查看汇总表格
cat test_logs_*/test_report.md | grep "|"
```

## 🔍 单独测试某一层

如果只想测试特定层级：

```bash
# Source环境
source /opt/ros/humble/setup.bash
source build_x86_shared/install/setup.bash

# 测试硬件层
ros2 launch system_bringup 1_hardware.launch.py

# 测试智能层（会自动启动硬件层和感知层）
ros2 launch system_bringup 3_intelligence.launch.py

# 测试特定平台
ros2 launch system_bringup qr_raspi.launch.py
```

## ⚙️ 测试参数

每个launch文件支持以下参数：

```bash
# 使用仿真时间
ros2 launch system_bringup 1_hardware.launch.py use_sim_time:=true

# 禁用某些功能（感知层）
ros2 launch system_bringup 2_perception.launch.py enable_audio:=false

# 启用跟随功能（智能层）
ros2 launch system_bringup 3_intelligence.launch.py enable_follow:=true

# 平台特定参数
ros2 launch system_bringup qr_raspi.launch.py enable_camera:=false
```

## 🐛 故障排查

### 问题1: 找不到 system_bringup 包

**解决方案**:
```bash
# 重新编译 core_layer
./build_layered.sh -c core_layer

# 重新source环境
source build_x86_shared/install/setup.bash

# 验证包是否存在
ros2 pkg list | grep system_bringup
```

### 问题2: 节点启动失败

**原因**: 某些节点的包还未编译

**解决方案**:
- 这是正常的，测试脚本会记录哪些节点存在/缺失
- 如需完整功能，需要编译对应层级：

```bash
# 编译硬件层（包含硬件层节点）
./build_layered.sh hardware_layer

# 编译感知层（包含硬件+感知层节点）
./build_layered.sh perception_layer

# 编译智能层（包含硬件+感知+智能层节点）
./build_layered.sh intelligence_layer
```

### 问题3: 配置文件未找到

**现象**: 日志中显示 "config file not found"

**解决方案**:
- 这是正常的，launch文件会自动降级使用默认参数
- 配置文件需要在编译对应层级后才会存在

### 问题4: Docker权限问题

**解决方案**:
```bash
# 确保使用sudo运行docker
sudo ./docker.sh run

# 或将当前用户加入docker组
sudo usermod -aG docker $USER
# 重新登录后生效
```

## 📖 详细文档

- **主README**: `README.md` - 项目总体说明
- **System Bringup**: `src/core_layer/src/system_bringup/README.md` - 详细使用文档
- **测试脚本**: `test_system_bringup.sh` - 自动化测试脚本源码

## 🎯 预期行为

### 正常情况

- ✅ Launch文件能成功解析和执行
- ✅ 能检测到ROS2环境变量
- ✅ 层级依赖正确（高层包含低层）
- ✅ 参数能正确传递

### 预期警告

- ⚠️ 某些节点包未找到（因为只编译了 core_layer）
- ⚠️ 配置文件未找到（会使用默认参数）
- ⚠️ 节点启动失败（因为可执行文件不存在）

这些警告是**预期的**，因为我们只编译了 core_layer 来测试 system_bringup 包本身的功能。

## 🔄 完整系统测试

如需测试完整功能（所有节点实际运行）：

```bash
# 1. 编译所有层
./build_layered.sh application_layer

# 2. 在Docker容器内运行测试
./docker.sh run

# 3. 在容器内执行
source /opt/ros/humble/setup.bash
source build_x86_shared/install/setup.bash
./test_system_bringup.sh
```

此时所有节点都应该能成功启动。

## 📈 测试指标

测试脚本会收集以下数据：

- **启动成功率**: Launch文件是否能成功解析
- **节点数量**: 检测到多少个ROS2节点
- **话题数量**: 检测到多少个ROS2话题
- **运行时长**: 每个测试运行15秒
- **日志完整性**: 每层都有完整的日志文件

## 🎓 学习要点

通过这个测试，你可以：

1. ✅ 理解 system_bringup 的分层架构
2. ✅ 学习如何在 core_layer 实现系统启动
3. ✅ 掌握 ROS2 launch文件的编写
4. ✅ 了解如何使用 Docker 进行测试
5. ✅ 熟悉分层编译和依赖管理

## 🤝 反馈

如果测试发现问题，请检查：

1. launch文件语法是否正确
2. 节点包名和可执行文件名是否匹配
3. 配置文件路径是否正确
4. 层级依赖关系是否合理

测试完成后，查看测试报告了解详细结果！
