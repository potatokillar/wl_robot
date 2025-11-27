# Jenkins CI 工具和脚本

> **⚠️ 注意**: 本目录包含 Jenkins CI 工具和脚本。
>
> **完整文档请查看**: [docs/ci-cd/](../docs/ci-cd/)
> - 📖 [Jenkins 配置指南](../docs/ci-cd/jenkins-setup.md) - 完整的初始配置
> - 📋 [Jenkins 使用指南](../docs/ci-cd/jenkins-usage.md) - 日常操作方法
> - 🔧 [故障排除指南](../docs/ci-cd/jenkins-troubleshooting.md) - 常见问题解决

本目录包含 qr_wl 项目的 Jenkins 持续集成工具脚本。

## 📁 文件说明

| 文件 | 用途 |
|------|------|
| `trigger_build.sh` | 触发 Jenkins 构建的便捷脚本 |
| `watch_latest_build.sh` | 实时监控最新构建状态 |
| `create-jenkins-job.sh` | 通过 API 自动创建 Jenkins 任务 |
| `job-config.xml` | Jenkins 任务配置文件模板 |
| `../Jenkinsfile` | Jenkins Pipeline 定义文件（项目根目录） |

## 🚀 快速开始

### 触发构建

```bash
# x86 架构构建
./jenkins/trigger_build.sh x86 debug

# ARM Debug 模式（QEMU 编译）
./jenkins/trigger_build.sh arm debug

# ARM Release 模式（Orin 硬件编译）
./jenkins/trigger_build.sh arm release
```

### 监控构建

```bash
# 实时监控最新构建
./jenkins/watch_latest_build.sh
```

### 查看 Jenkins UI

- **Jenkins 首页**: http://192.168.1.93:8080/
- **构建任务**: http://192.168.1.93:8080/job/qr-wl-build-ci

## 🔧 脚本使用说明

### trigger_build.sh

触发 Jenkins 构建任务。

**语法**:
```bash
./jenkins/trigger_build.sh <architecture> <build_mode>
```

**参数**:
- `architecture`: `x86` 或 `arm`
- `build_mode`: `debug` 或 `release`

**示例**:
```bash
# x86 架构，Debug 模式
./jenkins/trigger_build.sh x86 debug

# ARM 架构，Debug 模式（Jenkins QEMU）
./jenkins/trigger_build.sh arm debug

# ARM 架构，Release 模式（Orin 硬件编译）
./jenkins/trigger_build.sh arm release
```

### watch_latest_build.sh

实时监控最新构建状态，每 10 秒刷新一次。

**使用**:
```bash
./jenkins/watch_latest_build.sh
```

按 `Ctrl+C` 停止监控。

### create-jenkins-job.sh

通过 Jenkins API 自动创建或更新 Jenkins 任务配置。

**使用**:
```bash
./jenkins/create-jenkins-job.sh
```

**功能**:
- ✅ 自动通过 Jenkins API 创建任务
- ✅ 支持更新已有任务配置
- ✅ 处理 CSRF 保护（自动获取 crumb token）
- ✅ 自动打开浏览器显示任务页面

**前置条件**:
- Jenkins 服务器可访问 (http://192.168.1.93:8080)
- 已配置 Git 凭据 (ID: `git-cred`)
- Harbor 镜像仓库可访问

## 📚 完整文档

本目录仅包含工具脚本。完整的配置和使用文档请查看：

### CI/CD 文档

- **[Jenkins 配置指南](../docs/ci-cd/jenkins-setup.md)**
  - 初始安装配置
  - Pipeline 设置
  - 凭据配置
  - Harbor 镜像配置

- **[Jenkins 使用指南](../docs/ci-cd/jenkins-usage.md)**
  - 触发构建方法
  - 查看构建状态
  - 下载构建产物
  - 常用操作

- **[Jenkins 故障排除](../docs/ci-cd/jenkins-troubleshooting.md)**
  - Docker 访问配置
  - Git 凭据错误
  - 构建超时处理
  - Orin 板连接问题
  - 网络连接问题

### 其他相关文档

- **[CLAUDE.md](../CLAUDE.md)** - 项目架构和开发指南
- **[VERSION_MANAGEMENT.md](../VERSION_MANAGEMENT.md)** - 版本管理系统说明

## 💡 提示

1. 首次使用请先阅读 [Jenkins 配置指南](../docs/ci-cd/jenkins-setup.md)
2. 日常使用参考 [Jenkins 使用指南](../docs/ci-cd/jenkins-usage.md)
3. 遇到问题查看 [故障排除指南](../docs/ci-cd/jenkins-troubleshooting.md)
4. 脚本需要在项目根目录或 jenkins/ 目录下运行

## 🔗 相关链接

- **Jenkins 服务器**: http://192.168.1.93:8080/
- **构建任务**: http://192.168.1.93:8080/job/qr-wl-build-ci
- **Harbor 镜像仓库**: http://192.168.1.93/
- **Git 仓库**: http://192.168.1.55/ontology/qr_wl.git

---

**文档更新**: 2025-10-15
