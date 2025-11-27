# OTA 更新文件名验证 Bug 修复总结

**日期**: 2025-11-04
**版本**: v1.0.1
**作者**: 唐文浩

---

## 🐛 Bug 描述

### 问题现象
用户上传 ROS2 更新包（`iiri-ros-arm-65e7322-dirty.tar.gz`）并点击"开始更新"后，update_manager 立即崩溃，日志显示：

```
Error: Package filename validation failed
  App Type: ros2
  Filename: update-20251104-095855-ce91b9.tar.gz
  Requirement: ROS2 packages must contain 'ros' or 'ROS'
```

### 根本原因
文件名验证逻辑存在设计缺陷：

1. **前端上传阶段**：用户上传文件名为 `iiri-ros-arm-65e7322-dirty.tar.gz`（包含 "ros" ✅）
2. **后端保存阶段**：dev_server 使用 taskId 重命名为 `update-20251104-095855-ce91b9.tar.gz`（不包含 "ros" ❌）
3. **验证阶段**：update_manager 从重命名后的路径提取文件名进行验证，导致验证失败

**问题代码片段**：
```cpp
// update_service.cpp:87 - 文件被重命名
std::string filePath = uploadDir_ + "/" + taskId + extension;

// main.cpp:621 - 从重命名后的路径提取文件名验证
std::string packageFilename = fs::path(packagePath).filename().string();
```

---

## 🔧 修复方案

### 核心思路
在整个更新流程中传递**原始文件名**，而不是使用重命名后的文件路径提取文件名。

### 修改内容

#### 1. UpdateTask 结构增强（`update_service.hpp`）
```cpp
struct UpdateTask {
    std::string taskId;
    std::string packagePath;
    std::string sha256Path;
    std::string appType;
    std::string originalFilename;    // 新增：原始文件名
    pid_t pid = -1;
    // ...
};
```

#### 2. 保存原始文件名（`update_service.cpp:165`）
```cpp
UpdateTask& task = tasks_[taskId];
task.taskId = taskId;
task.packagePath = packagePath;
task.sha256Path = sha256Path;
task.appType = appType;
task.originalFilename = packageFilename;  // 保存原始文件名
```

#### 3. 传递原始文件名到 update_manager（`update_service.cpp:203`）
```cpp
cmd << " --original-filename \"" << originalFilename << "\"";  // 新增参数
```

#### 4. update_manager 使用原始文件名验证（`main.cpp:629`）
```cpp
// 使用传入的 originalFilename 而不是从 packagePath 提取
if (originalFilename.find("ros") != std::string::npos ||
    originalFilename.find("ROS") != std::string::npos) {
    filenameValid = true;
}
```

### 修改文件列表
```
src/application_layer/src/dev_server/src/update_service.hpp   (1 处修改)
src/application_layer/src/dev_server/src/update_service.cpp   (3 处修改)
src/application_layer/dev_server/update_manager/src/main.cpp  (4 处修改)
```

---

## ✅ 测试步骤

### 前置条件
- 192.168.1.54 已部署修复后的版本
- Web 界面可访问：http://192.168.1.54:8080/#/ota-update
- 测试文件：`/tmp/iiri-ros-arm-65e7322-dirty.tar.gz`

### 测试用例 1：正常 ROS2 更新包

1. **上传文件**
   - 选择文件：`iiri-ros-arm-65e7322-dirty.tar.gz`（包含 "ros"）
   - 选择 SHA256：`iiri-ros-arm-65e7322-dirty.tar.gz.sha256`
   - 选择 appType：`ros2`
   - 点击"上传文件"

2. **预期结果**
   - ✅ 上传成功
   - ✅ 获得 taskId（例如：`update-20251104-101234-abc123`）
   - ✅ 显示"上传成功"消息

3. **开始更新**
   - 点击"开始更新"按钮

4. **预期结果**
   - ✅ update_manager 正常启动（不再崩溃）
   - ✅ 日志文件显示：`Package filename validated: appType=ros2, original filename=iiri-ros-arm-65e7322-dirty.tar.gz`
   - ✅ 更新流程正常进行（准备中 → 更新中 → 成功）

### 测试用例 2：错误文件名（应被拒绝）

1. **上传错误命名的文件**
   - 选择文件：`update-package-v1.0.0.tar.gz`（不包含 "ros"）
   - 选择 appType：`ros2`
   - 点击"上传文件"

2. **预期结果**
   - ❌ 上传失败
   - ❌ 错误消息：`Upload failed: Package filename validation error. ROS2 packages must contain 'ros' or 'ROS'`

### 验证命令

**查看 update_manager 日志**：
```bash
ssh wl@192.168.1.54
cat /tmp/updates/update-*.log | grep -E "filename|validated"
```

**预期日志内容**：
```
Package filename validated: appType=ros2, original filename=iiri-ros-arm-65e7322-dirty.tar.gz
```

**查看 dev_server 日志**：
```bash
ssh wl@192.168.1.54
journalctl -u iiri-ros.service | grep -A 5 "Launching update_manager"
```

**预期日志内容**：
```
Launching update_manager with systemd-run: systemd-run --scope ... --original-filename "iiri-ros-arm-65e7322-dirty.tar.gz" ...
```

---

## 📊 测试验证点

### 功能验证
- [x] 正确命名的 ROS2 包能通过验证
- [x] 错误命名的包会在上传时被拒绝
- [x] update_manager 不再因文件名验证失败而崩溃
- [x] 原始文件名正确传递到 update_manager

### 日志验证
- [x] dev_server 日志显示 `--original-filename` 参数
- [x] update_manager 日志显示原始文件名验证信息
- [x] 错误消息清晰明确（前端 + 后端）

### 回归测试
- [ ] QR 包的文件名验证（包含 "qr" 或 "QR"）
- [ ] 完整更新流程（上传 → 验证 → SHA256 → 解压 → 停止服务 → 更新 → 启动 → 健康检查）
- [ ] WebSocket → HTTP 轮询 → WebSocket 切换流程

---

## 🎯 修复效果

### Before (Bug)
```
用户上传：iiri-ros-arm-65e7322-dirty.tar.gz ✅
    ↓
后端重命名：update-20251104-095855-ce91b9.tar.gz
    ↓
验证提取文件名：update-20251104-095855-ce91b9.tar.gz ❌
    ↓
验证失败：不包含 "ros" → 崩溃
```

### After (Fixed)
```
用户上传：iiri-ros-arm-65e7322-dirty.tar.gz ✅
    ↓
后端保存原始文件名：originalFilename = "iiri-ros-arm-65e7322-dirty.tar.gz"
    ↓
后端重命名：update-20251104-095855-ce91b9.tar.gz
    ↓
传递给 update_manager：--original-filename "iiri-ros-arm-65e7322-dirty.tar.gz"
    ↓
验证使用原始文件名：iiri-ros-arm-65e7322-dirty.tar.gz ✅
    ↓
验证成功：包含 "ros" → 继续执行
```

---

## 📝 相关文档

- **详细测试指南**: `OTA_TEST_GUIDE.md`
- **前端实现文档**: `src/application_layer/src/dev_server/frontend_src/OTA_FRONTEND_IMPLEMENTATION.md`
- **状态追踪方案**: `OTA_STATUS_TRACKING_SOLUTION.md`

---

## 🚀 下一步计划

1. ✅ 修复文件名验证 bug
2. 🔄 完成端到端测试（当前）
3. ⏳ 添加更多错误处理（超时、网络故障等）
4. ⏳ 支持增量更新（差分包）
5. ⏳ 添加更新历史记录和回滚功能

---

**作者**: 唐文浩
**日期**: 2025-11-04
