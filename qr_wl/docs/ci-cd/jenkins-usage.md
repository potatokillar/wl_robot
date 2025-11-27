# Jenkins 日常使用指南

> **文档位置**: `docs/ci-cd/jenkins-usage.md`
> **最后更新**: 2025-10-15

## 🎯 快速总结

Jenkins 任务 `qr-wl-build-ci` 已配置完成，提供自动化编译和打包功能。

**Jenkins 地址**: http://192.168.1.93:8080/job/qr-wl-build-ci

## 📋 任务信息

### 构建参数

| 参数 | 选项 | 默认值 | 说明 |
|------|------|--------|------|
| **ARCHITECTURE** | x86, arm | x86 | 目标架构 |
| **BUILD_MODE** | debug, release | debug | ARM 构建模式 |

**BUILD_MODE 说明**（仅 ARM 架构生效）:
- **debug**: Jenkins QEMU 编译（快速测试，有限制）
- **release**: Orin 硬件编译（生产发布，无限制）

## 🚀 触发构建

### 方式 1: Web UI（推荐）

1. 打开浏览器访问：http://192.168.1.93:8080/job/qr-wl-build-ci
2. 点击 **"Build with Parameters"**
3. 选择参数
4. 点击 **"Build"**

### 方式 2: 使用触发脚本

```bash
# x86 架构，Debug 模式
./jenkins/trigger_build.sh x86 debug

# ARM Debug 模式（QEMU 编译）
./jenkins/trigger_build.sh arm debug

# ARM Release 模式（Orin 硬件编译）
./jenkins/trigger_build.sh arm release
```

### 方式 3: 自动触发

代码推送到 `main` 分支后，Jenkins 会在 2 小时内自动触发构建。

## 📊 查看构建状态

### Web UI 查看

**构建历史**：
- 任务主页: http://192.168.1.93:8080/job/qr-wl-build-ci
- 颜色表示状态：
  - 🔵 蓝色：成功
  - 🔴 红色：失败
  - ⚪ 灰色：未执行
  - 🔵⏳ 闪烁：正在构建

**构建详情**：
- 点击构建编号查看详情
- **Console Output**: 完整日志
- **Changes**: 代码变更
- **Artifacts**: 下载构建产物

### 命令行查看

```bash
# 查看最新构建状态
curl -s "http://admin:westlake@192.168.1.93:8080/job/qr-wl-build-ci/lastBuild/api/json" \
  | python3 -c "import sys, json; d=json.load(sys.stdin); print(f\"结果: {d['result']}, 编号: {d['number']}\")"
```

### 使用脚本监控

```bash
# 实时监控最新构建
./jenkins/watch_latest_build.sh
```

## 🔄 构建流程

```
1. 清理工作空间
   ↓
2. 拉取代码 (main 分支)
   ↓
3. 检查 Docker
   ↓
4. 拉取 Docker 镜像
   ↓
5. 编译 (x86本地 / ARM Debug本地 / ARM Release远程Orin)
   ↓
6. 打包部署包
   ↓
7. 归档产物
```

**总耗时**：
- x86: 约 5-10 分钟
- ARM Debug: 约 10-15 分钟
- ARM Release: 约 5-10 分钟

## 📦 下载构建产物

### 从 Web UI 下载

1. 访问构建页面
2. 点击左侧 **"Build Artifacts"**
3. 下载文件：
   - `iiri-qr-{arch}-{version}.tar.gz` - 部署包
   - `*.sha256` - 校验文件

### 使用 wget 下载

```bash
# 下载最新成功构建的部署包
wget http://admin:westlake@192.168.1.93:8080/job/qr-wl-build-ci/lastSuccessfulBuild/artifact/deploy_packages/iiri-qr-*.tar.gz

# 验证完整性
sha256sum -c *.sha256
```

## ⚙️ 常用操作

### 停止正在运行的构建

1. 访问构建页面
2. 点击左侧 **"X"** 按钮（Abort）

### 重新触发上一次构建

1. 访问构建页面
2. 点击左侧 **"Rebuild"**

## 💡 部署到树莓派

```bash
# 1. 下载部署包
wget http://admin:westlake@192.168.1.93:8080/job/qr-wl-build-ci/lastSuccessfulBuild/artifact/deploy_packages/iiri-qr-arm-*.tar.gz

# 2. 上传到树莓派
scp iiri-qr-arm-*.tar.gz wl@192.168.1.54:/home/wl/autorun/

# 3. 解压并部署
ssh wl@192.168.1.54 "cd /home/wl/autorun && tar -xzf iiri-qr-*.tar.gz && cd iiri-qr-* && ./install.sh"
```

## 🔗 相关文档

- **[Jenkins 配置指南](jenkins-setup.md)** - 详细配置说明
- **[故障排除指南](jenkins-troubleshooting.md)** - 常见问题解决
- **[Jenkinsfile](../../Jenkinsfile)** - Pipeline 配置

---

**最后更新**: 2025-10-15
