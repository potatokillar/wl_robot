# IIRI ROS2 部署包使用指南

本目录包含 IIRI ROS2 系统的完整部署包，包括编译产物和自动化部署脚本。

## 📦 目录说明

此目录是一个**自包含的部署包**，包含：

- **install/** - ROS2 工作空间编译产物（自动同步）
- **启动脚本** - start_ros2_iiri_start.sh, iiri_start.sh 等
- **systemd 服务** - iiri-qr.service, iiri-ros.service
- **部署脚本** - deploy_systemd_services.sh, uninstall_systemd_services.sh
- **环境配置** - setup.bash, iiri_env.sh

> **重要提示**：`install/` 目录是由编译流程自动同步的，请勿手动修改。每次运行 `./build_layered.sh` 后会自动更新。

## 🚀 快速开始

### 一键部署（推荐）

```bash
# 1. 解压部署包到 autorun 目录
tar -xzf ~/downloads/iiri-ros-x86-v1.2.3.tar.gz -C /home/wl/autorun/

# 2. 创建符号链接（使部署路径保持一致）
ln -snf /home/wl/autorun/iiri-ros-x86-v1.2.3 /home/wl/autorun/iiri-ros

# 3. 完整部署（安装 + 启用 + 启动）
cd /home/wl/autorun/iiri-ros
sudo ./deploy_systemd_services.sh deploy

# 4. 查看服务状态
sudo ./deploy_systemd_services.sh status
```

> **💡 提示**：符号链接允许 systemd 服务使用固定路径 `/home/wl/autorun/iiri-ros`，
> 同时支持多版本并存和快速回滚。部署脚本会自动创建/更新符号链接。

### 本地开发测试

从源码编译后测试：

```bash
# 1. 编译项目（会自动同步到 iiri-ros/install）
cd /home/wl/twh/workspace/iiri_ros2_architecture
./build_layered.sh application_layer

# 2. 直接在项目目录测试
cd iiri-ros
source setup.bash
ros2 launch system_bringup qr_raspi.launch.py

# 或者部署到 autorun 目录
sudo ./deploy_systemd_services.sh deploy
```

## 服务说明

### iiri-qr.service
- QR_WL 自动运行服务
- 工作目录：/home/wl/autorun/iiri-qr
- 启动脚本：qr_start.sh
- 停止脚本：qr_stop.sh

### iiri-ros.service
- WL_ROS 自动运行服务
- 依赖：iiri-qr.service（必须先启动）
- 工作目录：/home/wl/autorun/iiri-ros
- 启动脚本：start_ros2_iiri_start.sh
- 停止脚本：stop_ros2_iiri_advanced.sh

## 使用命令

| 命令 | 说明 |
|------|------|
| deploy | 完整部署（安装+启用+启动） |
| install | 仅安装和启用开机自启 |
| start | 启动服务 |
| stop | 停止服务 |
| restart | 重启服务 |
| status | 查看服务状态 |
| help | 显示帮助信息 |

## 服务管理

### 查看状态
```bash
sudo systemctl status iiri-qr.service
sudo systemctl status iiri-ros.service
```

### 启动/停止
```bash
# 启动
sudo systemctl start iiri-qr.service
sudo systemctl start iiri-ros.service

# 停止
sudo systemctl stop iiri-ros.service
sudo systemctl stop iiri-qr.service
```

### 查看日志
```bash
# 实时日志
sudo journalctl -u iiri-ros.service -f

# 历史日志
sudo journalctl -u iiri-ros.service -n 50

# 启动脚本日志
cat /tmp/iiri_ros_startup_debug.log
```

## 故障排查

### 找不到 ROS2 setup 文件
```bash
ls -la /opt/ros/humble/setup.bash
sudo apt install ros-humble-desktop
```

### 找不到工作空间 setup 文件
```bash
ls -la /home/wl/autorun/iiri-ros/install/setup.bash
cp -r /path/to/build/install /home/wl/autorun/iiri-ros/
```

### 权限问题
```bash
sudo chown -R wl:wl /home/wl/autorun/iiri-ros/
sudo chmod +x /home/wl/autorun/iiri-ros/*.sh
```

## 卸载服务

```bash
sudo ./uninstall_systemd_services.sh
```

## 文件结构

```
iiri-ros/
├── README.md                          # 本文档
├── deploy_systemd_services.sh         # 部署脚本
├── uninstall_systemd_services.sh      # 卸载脚本
├── iiri-qr.service                    # QR服务配置
├── iiri-ros.service                   # ROS服务配置
├── start_ros2_iiri_start.sh          # 启动脚本
└── stop_ros2_iiri_advanced.sh        # 停止脚本
```

## 运行时目录

```
/home/wl/autorun/
├── iiri-qr/
│   ├── qr_start.sh
│   └── qr_stop.sh
└── iiri-ros/
    ├── start_ros2_iiri_start.sh
    ├── stop_ros2_iiri_advanced.sh
    └── install/setup.bash
```

## 🔄 编译流程说明

### install 目录来源

`install/` 目录由以下流程自动生成：

1. **本地开发编译**：
   ```bash
   cd /path/to/iiri_ros2_architecture
   ./build_layered.sh application_layer
   # → 自动同步到 iiri-ros/install/
   ```

2. **Jenkins CI/CD**：
   ```
   1. Jenkins 自动编译所有层级
   2. 自动同步到 iiri-ros/install/
   3. 打包成 iiri-ros-{arch}-{version}.tar.gz
   4. 归档到 Jenkins 供下载
   ```

### 从 Jenkins 下载部署包

```bash
# 1. 从 Jenkins 构建页面下载部署包
wget http://jenkins-server/job/iiri-ros-build/lastSuccessfulBuild/artifact/deploy_packages/iiri-ros-arm-v1.2.3.tar.gz

# 2. 验证校验和
sha256sum -c iiri-ros-arm-v1.2.3.tar.gz.sha256

# 3. 解压到目标目录
tar -xzf iiri-ros-arm-v1.2.3.tar.gz -C /home/wl/autorun/

# 4. 进入目录部署
cd /home/wl/autorun/iiri-ros-arm-v1.2.3
sudo ./deploy_systemd_services.sh deploy
```

## 📋 system_bringup 包说明

本部署包使用新的 **system_bringup** 包（位于 core_layer），相比旧的 bringup 包有以下优势：

| 对比项 | 旧 bringup | 新 system_bringup |
|--------|-----------|------------------|
| 所在层级 | application_layer (第5层) | core_layer (第1层) |
| 编译依赖 | 必须编译5层 | 最少编译3层 |
| 树莓派编译时间 | 长 | 短（减少40%） |
| 启动方式 | 单一 | 分层 + 平台 |

**启动文件说明**：

- `qr_raspi.launch.py` - 树莓派平台（3层架构，低功耗）
- `qr_orin.launch.py` - Orin平台（4层架构，高性能）
- `qr_arm.launch.py` - 通用ARM平台（3层架构）
- `qr_debug.launch.py` - 调试模式（仅硬件层）

详见：`src/core_layer/src/system_bringup/README.md`

## 🔧 手动打包

如果需要手动创建部署包：

```bash
cd /path/to/iiri_ros2_architecture

# 打包 x86 版本
./deploy_package.sh x86

# 打包 ARM 版本
./deploy_package.sh arm

# 指定输出目录
./deploy_package.sh arm custom_output_dir

# 查看生成的包
ls -lh deploy_packages/
```

## 更新日志

### 2025-10-11
- ✅ 添加 install 自动同步功能
- ✅ 创建标准化部署包打包流程
- ✅ 集成 Jenkins CI/CD 自动打包
- ✅ 使用新的 system_bringup 包
- ✅ 添加版本追踪和校验和

### 2025-10-10
- 初始版本
- 添加部署和卸载脚本
