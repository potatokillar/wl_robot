# OTA 更新功能测试指南

**日期**: 2025-11-04
**版本**: v1.0.0
**目标机器**: 192.168.1.54 (wl@192.168.1.54)

## 📦 部署内容总结

### 已部署的文件

#### 1. 更新包（位于 /tmp/）
```bash
/tmp/iiri-ros-arm-65e7322-dirty.tar.gz        # 162M ROS2 集群更新包
/tmp/iiri-ros-arm-65e7322-dirty.tar.gz.sha256 # SHA256 校验文件
```

#### 2. update_manager（位于 /home/wl/autorun/update_manager/）
```bash
/home/wl/autorun/update_manager/update_manager  # 182K 更新管理器二进制
/home/wl/autorun/update_manager/config.conf     # 配置文件
```

#### 3. 当前运行的服务
```bash
iiri-ros.service: ● active (running) 运行中
  - 启动时间: 16小时前
  - 内存使用: 577.9M
  - 进程数: 147
```

## 🚀 测试前准备

### 1. 访问 Web 界面
```
URL: http://192.168.1.54:8080/
导航到: OTA Update 页面
```

### 2. 准备测试文件

**正确命名的测试包**：
```bash
# ROS2 包（应该成功）
✓ iiri-ros-arm-65e7322-dirty.tar.gz  # 包含 "ros"

# QR 包（用于后续测试，如果有的话）
✓ iiri-qr-v1.0.0.tar.gz              # 包含 "qr"
```

**错误命名的测试包（用于验证检测）**：
```bash
# 错误命名（应该被拒绝）
✗ update-v1.0.0.tar.gz               # 缺少 ros/qr 关键字
✗ iiri-qr-v1.0.0.tar.gz + appType=ros2  # 文件名不匹配
```

## 📋 测试步骤

### 测试 1：验证文件名检测（必测）

#### 步骤：
1. 打开浏览器访问 `http://192.168.1.54:8080/#/ota-update`
2. 选择错误命名的文件（如没有包含 "ros" 的文件）
3. 选择 SHA256 校验文件
4. 选择 appType = "ros2"
5. 点击"上传文件"

#### 预期结果：
```
❌ 上传失败
错误消息：Upload failed: Package filename validation error.
          ROS2 packages must contain 'ros' or 'ROS',
          QR packages must contain 'qr' or 'QR'.
```

### 测试 2：正常更新流程（核心测试）

#### 步骤：
1. 选择正确命名的文件：`iiri-ros-arm-65e7322-dirty.tar.gz`
2. 选择对应的 SHA256 文件：`iiri-ros-arm-65e7322-dirty.tar.gz.sha256`
3. 选择 appType = "ros2"
4. 点击"上传文件"
5. 等待上传成功（获得 taskId）
6. 点击"开始更新"

#### 预期行为（阶段性）：

**阶段 1: WebSocket 实时模式（前 30 秒）**
```
✓ 连接状态：WebSocket 已连接（绿色）
✓ 进度条：0% → 10% → 20% → 30% ...
✓ 状态：准备中 → 更新中
✓ 当前阶段：
  - "Preparing update..."
  - "Verifying SHA256 checksum..."
  - "Extracting package..."
  - "Stopping service..."
```

**阶段 2: ROS2 停止（WebSocket 断开）**
```
⚠️  连接状态：HTTP 轮询模式（黄色）
⚠️  提示框：
   标题：ROS2 服务正在更新中
   内容：设备正在重启，已切换到 HTTP 轮询模式获取更新状态。
         请耐心等待，这可能需要几分钟...
   进度条：轮询中 (1/60) → (2/60) → ...
```

**阶段 3: ROS2 重启（恢复连接）**
```
✓ 连接状态：WebSocket 已连接（绿色）
✓ 停止轮询
✓ 显示最终结果：
   - 进度：100%
   - 状态：成功（绿色）
   - 消息：更新成功！用时 XXX 秒
```

### 测试 3：验证 systemd-run 进程隔离（高级）

在更新过程中，SSH 到机器验证 update_manager 独立运行：

```bash
# 在更新开始后 30 秒左右执行
ssh wl@192.168.1.54

# 查看 update_manager 进程
ps aux | grep update_manager
# 应该看到：systemd-run ... update_manager ...

# 查看 systemd scope 单元
systemctl status update-manager-*.scope

# 验证 ROS2 停止后 update_manager 仍在运行
systemctl status iiri-ros.service  # 应该显示 inactive
ps aux | grep update_manager       # 应该仍然存在

# 查看更新日志
tail -f /tmp/updates/update-*.log
```

### 测试 4：超时处理（可选）

如果更新超过 5 分钟（60 次 × 5 秒轮询）：

#### 预期结果：
```
❌ 超时错误
消息：更新状态查询超时，请手动刷新页面或检查设备状态
建议：检查设备网络连接和服务状态
```

## 🔍 验证点检查表

### 前端 UI 验证
- [ ] 连接状态标签正确显示（WebSocket/HTTP 轮询）
- [ ] 进度条实时更新
- [ ] 状态消息清晰易懂
- [ ] 错误提示详细明确
- [ ] 文件名验证提示显示
- [ ] 轮询进度条显示重试次数

### 后端验证
- [ ] update_manager 使用 systemd-run 启动
- [ ] 进程独立于 iiri-ros.service
- [ ] ROS2 停止期间 update_manager 继续运行
- [ ] 状态文件正确写入 `/var/run/update_status.json`
- [ ] 日志文件创建在 `/tmp/updates/`

### 连接切换验证
- [ ] WebSocket 断开时自动切换到 HTTP 轮询
- [ ] HTTP 轮询期间能获取更新状态
- [ ] ROS2 重启后自动恢复 WebSocket
- [ ] 超时后停止轮询并显示错误

## 📊 关键日志位置

### 前端日志（浏览器开发者工具）
```javascript
// 控制台输出
[OTA] WebSocket 收到更新状态: {...}
[OTA] HTTP 轮询尝试 1/60
[OTA] 更新完成，尝试恢复 WebSocket 连接
```

### 后端日志
```bash
# update_manager 日志
/tmp/updates/update-YYYYMMDD-HHMMSS-*.log

# 状态文件
/var/run/update_status.json

# systemd 日志
journalctl -u iiri-ros.service -f
journalctl -f | grep update-manager
```

## 🐛 故障排查

### 问题 1：上传失败
**症状**：点击上传后立即失败
**检查**：
- 浏览器开发者工具 Network 标签
- 查看 HTTP 状态码（应该是 400）
- 查看响应消息

### 问题 2：WebSocket 不断开
**症状**：更新过程中一直显示 WebSocket 连接
**原因**：可能 ROS2 未正确停止
**检查**：
```bash
systemctl status iiri-ros.service
ps aux | grep ros2
```

### 问题 3：HTTP 轮询无响应
**症状**：切换到轮询模式后没有进度更新
**检查**：
- 浏览器 Network 标签查看 `/api/update/current-status` 请求
- 查看状态文件：`cat /var/run/update_status.json`
- 检查 update_manager 是否在运行

### 问题 4：更新卡住不动
**症状**：进度停在某个百分比不变
**检查**：
```bash
# 查看 update_manager 进程
ps aux | grep update_manager

# 查看日志
tail -f /tmp/updates/update-*.log

# 检查状态文件
watch -n 1 cat /var/run/update_status.json
```

## 📝 测试报告模板

完成测试后，请填写以下报告：

```markdown
## OTA 更新测试报告

**测试日期**: YYYY-MM-DD
**测试人员**: [姓名]
**测试版本**: v1.0.0

### 测试结果总结
- [ ] 文件名验证：通过 / 失败
- [ ] 正常更新流程：通过 / 失败
- [ ] WebSocket → HTTP 切换：通过 / 失败
- [ ] HTTP → WebSocket 恢复：通过 / 失败

### 详细测试记录

#### 测试 1：文件名验证
- 上传错误文件名：[文件名]
- 错误消息是否正确：是 / 否
- 截图：[附上]

#### 测试 2：完整更新流程
- 开始时间：[HH:MM:SS]
- 结束时间：[HH:MM:SS]
- 总用时：[XXX 秒]
- WebSocket 断开时间点：[HH:MM:SS]
- HTTP 轮询持续时间：[XXX 秒]
- 最终状态：成功 / 失败
- 截图：[附上关键阶段截图]

### 问题记录
1. [问题描述]
   - 复现步骤
   - 错误消息
   - 解决方案

### 建议改进
1. [建议内容]
```

## 🎯 成功标准

✅ **测试通过标准**：
1. 文件名验证能正确拒绝不符合规则的文件
2. 正常更新流程能从开始到结束完整运行
3. WebSocket 断开后能自动切换到 HTTP 轮询
4. ROS2 重启后能自动恢复 WebSocket 连接
5. 更新完成后显示正确的成功/失败消息
6. update_manager 能独立于 ROS2 运行
7. 超时保护机制工作正常

## 📞 支持联系

如有问题请联系：
- **开发者**: 唐文浩
- **文档**: `/home/wl/twh/workspace/iiri_ros2_architecture/src/application_layer/src/dev_server/frontend_src/OTA_FRONTEND_IMPLEMENTATION.md`
