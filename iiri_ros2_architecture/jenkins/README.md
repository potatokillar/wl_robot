# Jenkins CI 工具和脚本

> **⚠️ 注意**: 本目录包含 Jenkins CI 工具和脚本。
>
> **完整文档请查看**: [docs/ci-cd/](../docs/ci-cd/)
> - 📖 [Jenkins 配置指南](../docs/ci-cd/jenkins-setup.md) - 完整的初始配置
> - 📋 [Jenkins 使用指南](../docs/ci-cd/jenkins-usage.md) - 日常操作方法
> - 🔧 [故障排除指南](../docs/reference/jenkins-troubleshooting.md) - 常见问题解决

本目录包含 IIRI ROS2 分层架构项目的 Jenkins 持续集成工具脚本。

## 📁 文件说明

| 文件 | 用途 |
|------|------|
| `create_job.py` | Python 脚本：自动创建/更新 Jenkins 任务 |
| `trigger_build.sh` | 触发 Jenkins 构建的便捷脚本 |
| `watch_latest_build.sh` | 实时监控最新构建状态 |
| `job-config.xml` | Jenkins 任务配置模板（XML 格式） |
| `create-jenkins-job.sh` | Bash 脚本：创建 Jenkins 任务（备用） |
| `../Jenkinsfile` | Jenkins Pipeline 定义文件（项目根目录） |

## 🚀 快速开始

### 触发构建

```bash
# x86 架构构建
./jenkins/trigger_build.sh x86 false

# ARM 架构构建
./jenkins/trigger_build.sh arm false

# 启用 Ceres 优化
./jenkins/trigger_build.sh x86 true
```

### 监控构建

```bash
# 实时监控最新构建
./jenkins/watch_latest_build.sh
```

### 查看 Jenkins UI

- **Jenkins 首页**: http://192.168.1.59:8081/
- **构建任务**: http://192.168.1.59:8081/job/iiri-layered-build-ci

## 🔧 脚本使用说明

### trigger_build.sh

触发 Jenkins 构建任务。

**语法**:
```bash
./jenkins/trigger_build.sh <architecture> <enable_ceres>
```

**参数**:
- `architecture`: `x86` 或 `arm`
- `enable_ceres`: `true` 或 `false`

**示例**:
```bash
# x86 架构，不启用 Ceres
./jenkins/trigger_build.sh x86 false

# ARM 架构，启用 Ceres
./jenkins/trigger_build.sh arm true
```

### watch_latest_build.sh

实时监控最新构建状态，每 10 秒刷新一次。

**使用**:
```bash
./jenkins/watch_latest_build.sh
```

按 `Ctrl+C` 停止监控。

### create_job.py

自动创建或更新 Jenkins 任务配置。

**使用**:
```bash
cd jenkins
python3 create_job.py
```

**注意**: 需要 Jenkins 服务器可访问且凭据已配置。

## 📚 完整文档

本目录仅包含工具脚本。完整的配置和使用文档请查看：

### CI/CD 文档

- **[Jenkins 配置指南](../docs/ci-cd/jenkins-setup.md)**
  - 初始安装配置
  - Pipeline 设置
  - 凭据配置
  - 插件安装

- **[Jenkins 使用指南](../docs/ci-cd/jenkins-usage.md)**
  - 触发构建方法
  - 查看构建状态
  - 下载构建产物
  - 常用操作

- **[Jenkins 故障排除](../docs/reference/jenkins-troubleshooting.md)**
  - vcstool 安装问题
  - Docker 访问配置
  - Git 凭据错误
  - 构建超时处理
  - 网络连接问题

### 其他相关文档

- **[部署指南](../docs/deployment/deployment-guide.md)** - 从 Jenkins 下载部署包的完整流程
- **[项目文档中心](../docs/README.md)** - 所有项目文档导航

## 💡 提示

1. 首次使用请先阅读 [Jenkins 配置指南](../docs/ci-cd/jenkins-setup.md)
2. 日常使用参考 [Jenkins 使用指南](../docs/ci-cd/jenkins-usage.md)
3. 遇到问题查看 [故障排除指南](../docs/reference/jenkins-troubleshooting.md)
4. 脚本需要在项目根目录或 jenkins/ 目录下运行

---

**Jenkins 地址**: http://192.168.1.59:8081/job/iiri-layered-build-ci
**文档更新**: 2025-10-13
