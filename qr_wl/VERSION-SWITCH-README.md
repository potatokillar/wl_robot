# IIRI 服务版本切换工具使用指南

## 📖 概述

`switch-version.sh` 是一个用于管理 IIRI-ROS 和 IIRI-QR 服务版本的命令行工具。它提供了便捷的版本切换、查看和回滚功能。

## 🚀 功能特性

- ✅ 支持 **iiri-ros** 和 **iiri-qr** 两个服务
- ✅ 列出所有已安装版本
- ✅ 查看当前运行版本
- ✅ 快速切换到指定版本
- ✅ 一键回滚到上一版本
- ✅ 自动停止/启动服务
- ✅ 智能版本号匹配
- ✅ 服务状态检查

## 📋 前置条件

- 需要 **root 权限**（使用 `sudo`）
- 版本部署在 `/home/wl/autorun/` 目录
- systemd 服务已正确配置

## 🔧 安装

将 `switch-version.sh` 复制到 `/home/wl/autorun/` 目录：

```bash
sudo cp switch-version.sh /home/wl/autorun/
sudo chmod +x /home/wl/autorun/switch-version.sh
```

## 📚 命令详解

### 1. 查看帮助

```bash
sudo /home/wl/autorun/switch-version.sh help
```

显示所有可用命令和使用示例。

---

### 2. 列出所有版本

#### 查看所有服务的版本

```bash
sudo /home/wl/autorun/switch-version.sh list
```

**输出示例：**
```
━━━ IIRI-ROS 已安装版本 ━━━

  [1] iiri-ros-x86-e9a7401-dirty  ← 当前版本
       版本: e9a7401-dirty
       构建时间: 2025-10-14 10:30:15
  [2] iiri-ros-x86-c513fad-dirty
       版本: c513fad-dirty
       构建时间: 2025-10-13 15:20:45

━━━ IIRI-QR 已安装版本 ━━━

  [1] iiri-qr-arm-19f5706-dirty  ← 当前版本
       版本: 19f5706-dirty
       构建时间: 2025-10-15 14:01:38
  [2] iiri-qr-arm-6950ad9-dirty
       版本: 6950ad9-dirty
       构建时间: 2025-10-15 13:56:01
```

#### 只查看特定服务的版本

```bash
# 只查看 ROS 版本
sudo /home/wl/autorun/switch-version.sh list ros

# 只查看 QR 版本
sudo /home/wl/autorun/switch-version.sh list qr
```

---

### 3. 查看当前版本

#### 查看所有服务的当前版本

```bash
sudo /home/wl/autorun/switch-version.sh current
```

**输出示例：**
```
━━━ IIRI-ROS 当前版本 ━━━
版本: iiri-ros-x86-e9a7401-dirty
路径: /home/wl/autorun/iiri-ros-x86-e9a7401-dirty
状态: ● 运行中

Project: iiri_ros2_architecture
Version: e9a7401-dirty
Architecture: x86
Build Date: 2025-10-14 10:30:15
...

━━━ IIRI-QR 当前版本 ━━━
版本: iiri-qr-arm-19f5706-dirty
路径: /home/wl/autorun/iiri-qr-arm-19f5706-dirty
状态: ● 运行中

Project: qr_wl
Version: 19f5706-dirty
Architecture: arm
Build Date: 2025-10-15 14:01:38
...
```

#### 只查看特定服务的当前版本

```bash
# 只查看 ROS 当前版本
sudo /home/wl/autorun/switch-version.sh current ros

# 只查看 QR 当前版本
sudo /home/wl/autorun/switch-version.sh current qr
```

---

### 4. 切换版本

#### 切换 iiri-ros 版本

```bash
# 使用完整版本号
sudo /home/wl/autorun/switch-version.sh ros iiri-ros-x86-c513fad-dirty

# 使用简短版本号（自动匹配）
sudo /home/wl/autorun/switch-version.sh ros c513fad

# 使用版本标签
sudo /home/wl/autorun/switch-version.sh ros v1.2.3
```

**切换流程：**
1. 停止 `iiri-ros.service`
2. 更新符号链接 `/home/wl/autorun/iiri-ros`
3. 启动 `iiri-ros.service`
4. 检查服务状态

**输出示例：**
```
━━━ 切换 IIRI-ROS 版本 ━━━

>>> 停止服务: iiri-ros.service
>>> 切换到: iiri-ros-x86-c513fad-dirty
>>> 启动服务: iiri-ros.service

✅ IIRI-ROS 版本切换成功！
```

#### 切换 iiri-qr 版本

```bash
# 使用完整版本号
sudo /home/wl/autorun/switch-version.sh qr iiri-qr-arm-6950ad9-dirty

# 使用简短版本号
sudo /home/wl/autorun/switch-version.sh qr 6950ad9

# 使用版本标签
sudo /home/wl/autorun/switch-version.sh qr v2.0.1
```

---

### 5. 回滚到上一版本

#### 回滚 iiri-ros

```bash
sudo /home/wl/autorun/switch-version.sh rollback ros
```

**功能说明：**
- 自动查找当前版本之前的最近一个版本
- 执行版本切换操作
- 如果没有旧版本，会提示错误

#### 回滚 iiri-qr

```bash
sudo /home/wl/autorun/switch-version.sh rollback qr
```

**输出示例：**
```
回滚到: iiri-qr-arm-6950ad9-dirty

━━━ 切换 IIRI-QR 版本 ━━━

>>> 停止服务: iiri-qr.service
>>> 切换到: iiri-qr-arm-6950ad9-dirty
>>> 启动服务: iiri-qr.service

✅ IIRI-QR 版本切换成功！
```

---

## 🎯 使用场景

### 场景 1：测试新版本

```bash
# 1. 查看当前版本
sudo /home/wl/autorun/switch-version.sh current qr

# 2. 列出所有版本
sudo /home/wl/autorun/switch-version.sh list qr

# 3. 切换到新版本
sudo /home/wl/autorun/switch-version.sh qr 19f5706

# 4. 测试功能...

# 5. 如果有问题，立即回滚
sudo /home/wl/autorun/switch-version.sh rollback qr
```

### 场景 2：灰度发布

```bash
# 在测试机器上部署新版本
tar -xzf iiri-qr-arm-v2.0.0.tar.gz -C /home/wl/autorun/
cd /home/wl/autorun/iiri-qr-arm-v2.0.0
sudo ./install.sh

# 新版本会自动成为当前版本
# 如果测试通过，在其他机器上重复部署
```

### 场景 3：多版本共存

```bash
# 系统中可以同时存在多个版本
/home/wl/autorun/
├── iiri-qr-arm-19f5706-dirty/
├── iiri-qr-arm-6950ad9-dirty/
├── iiri-qr -> iiri-qr-arm-19f5706-dirty/  # 当前版本

# 可以随时切换
sudo /home/wl/autorun/switch-version.sh qr 6950ad9
```

### 场景 4：故障恢复

```bash
# 发现服务异常
sudo systemctl status iiri-qr.service

# 查看日志
sudo journalctl -u iiri-qr.service -n 50

# 如果是新版本问题，快速回滚
sudo /home/wl/autorun/switch-version.sh rollback qr

# 查看回滚后状态
sudo /home/wl/autorun/switch-version.sh current qr
```

---

## 🛠 版本号匹配规则

工具支持多种版本号格式，按以下顺序匹配：

1. **完整目录名**
   ```bash
   sudo switch-version.sh qr iiri-qr-arm-19f5706-dirty
   ```

2. **架构 + 版本号**（自动添加前缀）
   ```bash
   sudo switch-version.sh qr arm-19f5706-dirty
   # 匹配: iiri-qr-arm-19f5706-dirty
   ```

3. **版本号**（模糊匹配）
   ```bash
   sudo switch-version.sh qr 19f5706
   # 匹配: iiri-qr-arm-19f5706-dirty 或 iiri-qr-x86-19f5706-dirty
   ```

4. **部分版本号**（模糊匹配）
   ```bash
   sudo switch-version.sh qr v2.0
   # 匹配所有包含 v2.0 的版本
   ```

⚠️ **注意**：如果模糊匹配找到多个版本，工具会列出所有匹配项并要求指定完整版本号。

---

## 📂 目录结构

```
/home/wl/autorun/
├── switch-version.sh           # 版本切换工具
│
├── iiri-ros -> iiri-ros-x86-e9a7401-dirty/  # ROS 符号链接（指向当前版本）
├── iiri-ros-x86-e9a7401-dirty/             # ROS 版本 1
│   ├── install.sh
│   ├── deploy_systemd_services.sh
│   ├── VERSION.txt
│   └── ...
├── iiri-ros-x86-c513fad-dirty/             # ROS 版本 2
│   └── ...
│
├── iiri-qr -> iiri-qr-arm-19f5706-dirty/   # QR 符号链接（指向当前版本）
├── iiri-qr-arm-19f5706-dirty/              # QR 版本 1
│   ├── install.sh
│   ├── deploy_qr_service.sh
│   ├── VERSION.txt
│   └── ...
└── iiri-qr-arm-6950ad9-dirty/              # QR 版本 2
    └── ...
```

**符号链接机制：**
- systemd 服务使用固定路径（如 `/home/wl/autorun/iiri-qr`）
- 符号链接指向实际的版本目录
- 切换版本 = 更新符号链接指向

---

## ⚙️ systemd 服务配置

### iiri-ros.service

```ini
[Service]
WorkingDirectory=/home/wl/autorun/iiri-ros
ExecStart=/home/wl/autorun/iiri-ros/start_ros2_iiri_start.sh
```

### iiri-qr.service

```ini
[Service]
WorkingDirectory=/home/wl/autorun/iiri-qr
ExecStart=/home/wl/autorun/iiri-qr/qr_start.sh
```

**关键点**：
- 服务配置使用**符号链接路径**，不是版本特定路径
- 版本切换时**不需要修改** systemd 服务文件
- 只需更新符号链接，重启服务即可

---

## ❗ 常见问题

### Q1: 切换版本失败怎么办？

**A:** 检查以下几点：
```bash
# 1. 确认目标版本存在
sudo /home/wl/autorun/switch-version.sh list qr

# 2. 检查 systemd 服务状态
sudo systemctl status iiri-qr.service

# 3. 查看详细日志
sudo journalctl -u iiri-qr.service -n 100

# 4. 手动停止服务
sudo systemctl stop iiri-qr.service

# 5. 重新尝试切换
sudo /home/wl/autorun/switch-version.sh qr <版本号>
```

### Q2: 如何清理旧版本？

**A:** 手动删除不需要的版本目录：
```bash
# 1. 先切换到其他版本
sudo /home/wl/autorun/switch-version.sh qr <新版本>

# 2. 确认当前版本
sudo /home/wl/autorun/switch-version.sh current qr

# 3. 删除旧版本目录（慎重！）
sudo rm -rf /home/wl/autorun/iiri-qr-arm-<旧版本号>

# 4. 验证
sudo /home/wl/autorun/switch-version.sh list qr
```

### Q3: 符号链接损坏怎么办？

**A:** 手动重建符号链接：
```bash
# 删除损坏的符号链接
sudo rm /home/wl/autorun/iiri-qr

# 重新创建符号链接到指定版本
sudo ln -s /home/wl/autorun/iiri-qr-arm-<版本号> /home/wl/autorun/iiri-qr

# 重启服务
sudo systemctl restart iiri-qr.service

# 验证
sudo /home/wl/autorun/switch-version.sh current qr
```

### Q4: 版本切换后服务无法启动？

**A:** 可能的原因和解决方法：
```bash
# 1. 检查版本目录完整性
ls -lh /home/wl/autorun/iiri-qr/

# 2. 检查必需文件
sudo /home/wl/autorun/iiri-qr/deploy_qr_service.sh status

# 3. 查看启动脚本权限
ls -l /home/wl/autorun/iiri-qr/qr_start.sh
ls -l /home/wl/autorun/iiri-qr/qr

# 4. 手动运行启动脚本测试
cd /home/wl/autorun/iiri-qr
sudo -u wl ./qr qr-linkV2-3.toml

# 5. 如果问题无法解决，回滚
sudo /home/wl/autorun/switch-version.sh rollback qr
```

### Q5: 如何同时切换两个服务的版本？

**A:** 分别切换：
```bash
# 先切换 QR
sudo /home/wl/autorun/switch-version.sh qr <版本号>

# 等待 QR 启动稳定
sleep 5

# 再切换 ROS
sudo /home/wl/autorun/switch-version.sh ros <版本号>

# 检查两个服务状态
sudo /home/wl/autorun/switch-version.sh current
```

---

## 🔐 安全注意事项

1. **始终需要 root 权限**
   - 工具会自动检查并提示使用 `sudo`

2. **切换版本会重启服务**
   - 会导致短暂的服务中断
   - 建议在低峰期或维护窗口执行

3. **保留多个版本备份**
   - 建议保留至少 2-3 个历史版本
   - 便于快速回滚

4. **测试后再部署**
   - 新版本先在测试环境验证
   - 确认无误后再部署到生产环境

---

## 📞 技术支持

如遇到问题，请联系：
- 作者：唐文浩
- 查看日志：`sudo journalctl -u iiri-qr.service -f`
- 查看服务状态：`sudo systemctl status iiri-qr.service`

---

## 📝 版本历史

- **v2.0** (2025-10-15)
  - 新增 iiri-qr 服务支持
  - 重构命令格式，更清晰直观
  - 增强版本匹配功能
  - 完善错误提示

- **v1.0** (2025-10-10)
  - 初始版本
  - 支持 iiri-ros 服务管理
