# Jenkins 持续集成配置指南

> **文档位置**: `docs/ci-cd/jenkins-setup.md`
> **最后更新**: 2025-10-15
> **作者**: 参考 iiri_ros2_architecture

本文档说明如何在 Jenkins 上配置 qr_wl 项目的自动化编译任务流。

## 功能概述

- ✅ 自动检测 Git 主分支代码变更
- ✅ 支持 x86 和 ARM 架构选择
- ✅ ARM 支持 Debug (QEMU) 和 Release (Orin硬件) 两种模式
- ✅ 自动打包部署包
- ✅ 自动清理构建产物节省空间
- ✅ 构建失败时及时通知

## 前置要求

### Jenkins 服务器要求

- Jenkins 版本: 2.x+
- Jenkins 地址: http://192.168.1.93:8080
- 已安装插件:
  - Git Plugin
  - Pipeline Plugin
  - Credentials Plugin
- Docker 环境已配置
- 可访问的 Docker 镜像仓库 (Harbor): http://192.168.1.93

### Docker 镜像要求

**重要**：需要在 Harbor 仓库中配置镜像标签

1. 登录 Harbor 仓库: http://192.168.1.93
2. 进入 `iiri/build_x86_arm_ros1` 仓库
3. 确保存在 `latest` 标签
4. 如果没有，给合适的镜像版本添加 `latest` 标签

**镜像地址**:
```
192.168.1.93/iiri/build_x86_arm_ros1:latest
```

### Git 仓库要求

- 主仓库: `http://192.168.1.55/ontology/qr_wl.git`
- 已在 Jenkins 中配置 Git 凭据 (ID: `git-cred`)
- Jenkinsfile 已提交到 main 分支

### Orin 板要求（ARM Release 模式）

- IP: 192.168.1.61
- 用户: wl
- 密码: 123456
- Docker 环境已安装
- 可通过 SSH 无密码访问（使用 sshpass）

验证 Orin 板连接:
```bash
# 测试 SSH 连接
sshpass -p '123456' ssh wl@192.168.1.61 "docker --version"

# 测试 Docker 镜像拉取
sshpass -p '123456' ssh wl@192.168.1.61 "docker pull 192.168.1.93/iiri/build_x86_arm_ros1:latest"
```

## 步骤 1: 配置 Git 凭据

如果尚未配置 Git 凭据：

1. 进入 Jenkins 首页
2. 点击 "Manage Jenkins" → "Manage Credentials"
3. 选择合适的域（通常是 "Global"）
4. 点击 "Add Credentials"
5. 配置如下:
   - **Kind**: Username with password
   - **Username**: `root` 或你的 GitLab 用户名
   - **Password**: 你的 GitLab 密码或访问令牌
   - **ID**: `git-cred` (必须使用这个 ID，或修改 Jenkinsfile)
   - **Description**: GitLab credentials for qr_wl

## 步骤 2: 创建 Pipeline 任务

### 方式 1: 通过 Web UI 创建（推荐）

1. 在 Jenkins 首页点击 "New Item"
2. 输入任务名称: `qr-wl-build-ci`
3. 选择 "Pipeline"
4. 点击 "OK"

### 方式 2: 通过脚本创建（半自动）

```bash
cd jenkins
./create-jenkins-job.sh
# 按照提示在 UI 中完成配置
```

## 步骤 3: 配置 Pipeline

在任务配置页面:

### General 配置

- ✅ 勾选 "Discard old builds"
  - Days to keep builds: 7
  - Max # of builds to keep: 10

### Build Triggers

选择触发方式之一:

**选项 A: 定时触发（推荐）**
- ✅ 勾选 "Build periodically"
- Schedule: `H */2 * * *` (每 2 小时检查一次)

**选项 B: Git 轮询触发**
- ✅ 勾选 "Poll SCM"
- Schedule: `H/5 * * * *` (每 5 分钟检查一次)

**选项 C: GitLab Webhook 触发（更即时）**
- ✅ 勾选 "Build when a change is pushed to GitLab"
- 在 GitLab 仓库设置中添加 Webhook:
  - URL: `http://192.168.1.93:8080/project/qr-wl-build-ci`
  - Trigger: Push events (仅 main 分支)

### Pipeline 配置

- **Definition**: Pipeline script from SCM
- **SCM**: Git
  - **Repository URL**: `http://192.168.1.55/ontology/qr_wl.git`
  - **Credentials**: 选择 `git-cred`
  - **Branch Specifier**: `*/main`
- **Script Path**: `Jenkinsfile`

## 步骤 4: Jenkinsfile 说明

Jenkinsfile 已包含在项目根目录，实现以下流程:

```groovy
1. 清理工作空间
2. 拉取主仓库代码
3. 检查 Docker 可用性
4. 拉取 Docker 镜像
5. 编译:
   - x86: Jenkins 本地 Docker 编译
   - ARM Debug: Jenkins QEMU 模拟编译
   - ARM Release: Orin 板硬件编译
6. 打包部署包
7. 归档构建产物
8. 清理（ARM Release 清理 Orin 构建目录）
```

### 构建参数

任务配置了以下参数（自动从 Jenkinsfile 读取）:

| 参数 | 类型 | 选项 | 默认值 | 说明 |
|------|------|------|--------|------|
| ARCHITECTURE | Choice | x86, arm | x86 | 目标架构 |
| BUILD_MODE | Choice | debug, release | debug | ARM 构建模式（仅ARM生效） |

**BUILD_MODE 说明**:
- **debug**: Jenkins QEMU 编译（快速测试，有限制）
- **release**: Orin 硬件编译（生产发布，无限制）

## 步骤 5: 测试运行

1. 保存任务配置
2. 点击 "Build with Parameters"
3. 选择架构 (x86 或 arm)
4. 选择构建模式 (debug 或 release)
5. 点击 "Build"

观察构建日志，确认编译成功。

### 第一次构建预期结果

- **x86 构建**: 约 5-10 分钟
- **ARM Debug 构建**: 约 10-15 分钟（QEMU 模拟较慢）
- **ARM Release 构建**: 约 5-10 分钟（Orin 硬件快速）

成功后会生成:
- `deploy_packages/iiri-qr-{arch}-{version}.tar.gz`
- `deploy_packages/iiri-qr-{arch}-{version}.tar.gz.sha256`

## 步骤 6: 配置通知 (可选)

### 邮件通知

在 Jenkinsfile 的 `post` 部分添加:

```groovy
post {
    success {
        emailext (
            subject: "构建成功: ${env.JOB_NAME} - ${env.BUILD_NUMBER}",
            body: "编译成功!\n\n查看详情: ${env.BUILD_URL}",
            to: "team@example.com"
        )
    }
    failure {
        emailext (
            subject: "构建失败: ${env.JOB_NAME} - ${env.BUILD_NUMBER}",
            body: "编译失败，请查看日志: ${env.BUILD_URL}console",
            to: "team@example.com"
        )
    }
}
```

### 钉钉/企业微信通知

安装相应插件后，在 `post` 部分添加通知代码。

## 故障排查

### 问题 1: Docker 镜像拉取失败

**现象**: 无法拉取 Docker 镜像

**解决**:
1. 确认 Jenkins 服务器可访问 `192.168.1.93`
2. 检查 Harbor 中镜像是否存在且有 `latest` 标签
3. 手动登录 Harbor 测试: `docker login 192.168.1.93`

### 问题 2: Git 凭据错误

**现象**: 无法克隆仓库

**解决**:
1. 检查 Jenkins 凭据配置中的用户名密码
2. 确认 GitLab 用户有仓库访问权限
3. 使用访问令牌代替密码

### 问题 3: Orin 板连接失败

**现象**: ARM Release 模式 SSH 连接失败

**解决**:
1. 确认 Orin 板在线: `ping 192.168.1.61`
2. 测试 SSH 访问: `sshpass -p '123456' ssh wl@192.168.1.61 "echo OK"`
3. 检查 Orin 板 Docker 环境: `sshpass -p '123456' ssh wl@192.168.1.61 "docker ps"`

### 问题 4: 构建超时

**现象**: 构建运行时间过长被中断

**解决**:
已在 Jenkinsfile 中配置 30 分钟超时:
```groovy
options {
    timeout(time: 30, unit: 'MINUTES')
}
```

如需调整，修改 Jenkinsfile 中的超时时间。

## 高级配置

### 多分支 Pipeline

如果需要同时监控多个分支:

1. 创建 "Multibranch Pipeline" 任务
2. 配置 Branch Sources 指向 Git 仓库
3. 设置 "Discover branches" 策略
4. Jenkinsfile 会自动应用到所有分支

### 并行构建

如果服务器资源充足，可以创建多个任务并行编译不同架构。

### 构建产物归档

已在 Jenkinsfile 中配置自动归档:
- 部署包: `deploy_packages/*.tar.gz`
- 校验文件: `deploy_packages/*.sha256`
- 版本信息: `deploy_packages/*/VERSION.txt`
- 编译日志: `build/**/log/**/*.log`

## 维护建议

1. **定期清理旧构建**: Jenkins 会根据配置自动清理
2. **监控磁盘空间**: 构建产物可能占用大量空间
3. **更新 Docker 镜像**: 定期更新构建镜像版本
4. **审查构建日志**: 定期检查警告和错误信息
5. **备份 Jenkinsfile**: 版本控制很重要

## 验证清单

完成配置后，确认以下项目:

- [ ] Jenkins 任务 `qr-wl-build-ci` 已创建
- [ ] Git 凭据 `git-cred` 已配置
- [ ] Harbor 镜像 `192.168.1.93/iiri/build_x86_arm_ros1:latest` 存在
- [ ] Orin 板 (192.168.1.61) SSH 连接正常
- [ ] 第一次 x86 构建成功
- [ ] 第一次 ARM Debug 构建成功
- [ ] 第一次 ARM Release 构建成功
- [ ] 部署包正常归档
- [ ] 触发脚本可用: `./jenkins/trigger_build.sh x86 debug`
- [ ] 监控脚本可用: `./jenkins/watch_latest_build.sh`

## 相关文档

- [Jenkinsfile 语法](https://www.jenkins.io/doc/book/pipeline/syntax/)
- [Jenkins 使用指南](jenkins-usage.md)
- [故障排除指南](jenkins-troubleshooting.md)
- [项目 CLAUDE.md](../../CLAUDE.md)
- [版本管理说明](../../VERSION_MANAGEMENT.md)

---

**配置完成后，Jenkins CI 将自动监控代码变更并触发构建。**
