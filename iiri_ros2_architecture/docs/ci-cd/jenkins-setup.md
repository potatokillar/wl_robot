# Jenkins 持续集成配置指南

> **文档位置**: `docs/ci-cd/jenkins-setup.md`
> **最后更新**: 2025-10-13
> **作者**: 唐文浩

本文档说明如何在 Jenkins 上配置自动化的分层编译任务流，用于持续检测主分支代码的编译问题。

## 功能概述

- ✅ 自动检测 Git 主分支代码变更
- ✅ 按层级顺序清理构建：core → hardware → perception → intelligence → application
- ✅ 支持 x86 和 ARM 架构选择
- ✅ 可选启用 Ceres 优化
- ✅ 自动清理构建产物节省空间
- ✅ 构建失败时及时通知

## 前置要求

### Jenkins 服务器要求

- Jenkins 版本: 2.x+
- 已安装插件:
  - Git Plugin
  - Pipeline Plugin
  - Credentials Plugin
- Docker 环境已配置
- 可访问的 Docker 镜像:
  - `192.168.1.93/iiri/build_x86_ros2:v1.4.3`
  - `192.168.1.93/iiri/build_arm_ros2:v1.4.2`

### Git 仓库要求

- 主仓库: `http://192.168.1.55/ontology/iiri_ros2_architecture.git`
- 已在 Jenkins 中配置 Git 凭据 (ID: `git-cred`)
- Jenkinsfile 已提交到主分支

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
   - **Description**: GitLab credentials

## 步骤 2: 创建 Pipeline 任务

### 方式 1: 通过 Web UI 创建

1. 在 Jenkins 首页点击 "New Item"
2. 输入任务名称: `iiri-layered-build-ci`
3. 选择 "Pipeline"
4. 点击 "OK"

### 方式 2: 通过 Jenkins CLI 创建 (可选)

```bash
# 使用 Jenkins CLI 创建任务
java -jar jenkins-cli.jar -s http://192.168.1.93:8080/ \
    -auth admin:westlake \
    create-job iiri-layered-build-ci < job-config.xml
```

## 步骤 3: 配置 Pipeline

在任务配置页面:

### General 配置

- ✅ 勾选 "Discard old builds"
  - Days to keep builds: 7
  - Max # of builds to keep: 10

### Build Triggers

选择触发方式之一:

**选项 A: Git 轮询触发 (推荐)**
- ✅ 勾选 "Poll SCM"
- Schedule: `H/5 * * * *` (每 5 分钟检查一次)

**选项 B: GitLab Webhook 触发 (更即时)**
- ✅ 勾选 "Build when a change is pushed to GitLab"
- 在 GitLab 仓库设置中添加 Webhook:
  - URL: `http://192.168.1.93:8080/project/iiri-layered-build-ci`
  - Trigger: Push events (仅 main 分支)

### Pipeline 配置

- **Definition**: Pipeline script from SCM
- **SCM**: Git
  - **Repository URL**: `http://192.168.1.55/ontology/iiri_ros2_architecture.git`
  - **Credentials**: 选择 `git-cred`
  - **Branch Specifier**: `*/main`
- **Script Path**: `Jenkinsfile`

### 参数配置

在 "This project is parameterized" 中添加:

1. **ARCHITECTURE** (Choice Parameter)
   - Choices:
     ```
     x86
     arm
     ```
   - Default: `x86`

2. **ENABLE_CERES** (Boolean Parameter)
   - Default Value: `false`
   - Description: 启用 Ceres 优化（用于 path_tracker）

## 步骤 4: Jenkinsfile 说明

Jenkinsfile 已包含在项目根目录，实现以下流程:

```groovy
1. 清理工作空间
2. 拉取主仓库代码
3. 使用 vcstool 导入所有分层仓库
4. 按顺序编译各层:
   - Core Layer
   - Hardware Layer
   - Perception Layer
   - Intelligence Layer
   - Application Layer
5. 编译成功/失败通知
6. 清理构建产物
```

每一层都使用清理构建 (`-c` 参数) 确保干净的编译环境。

## 步骤 5: 测试运行

1. 保存任务配置
2. 点击 "Build with Parameters"
3. 选择架构 (x86 或 arm)
4. 选择是否启用 Ceres
5. 点击 "Build"

观察构建日志，确认各层按顺序成功编译。

## 步骤 6: 配置通知 (可选)

### 邮件通知

在 Jenkinsfile 的 `post` 部分添加:

```groovy
post {
    success {
        emailext (
            subject: "构建成功: ${env.JOB_NAME} - ${env.BUILD_NUMBER}",
            body: "所有层编译成功!\n\n查看详情: ${env.BUILD_URL}",
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

### 问题 1: vcstool 导入失败

**现象**: `vcs import` 命令失败

**解决**:
```groovy
// 在 Jenkinsfile 中添加 vcstool 安装
sh '''
    pip3 install vcstool --user
    export PATH=$PATH:$HOME/.local/bin
    vcs import src < .repos
'''
```

### 问题 2: Docker 镜像拉取失败

**现象**: 无法拉取 Docker 镜像

**解决**:
1. 确认 Jenkins 服务器可访问 `192.168.1.93`
2. 检查 Docker 凭据配置
3. 手动登录 Harbor: `docker login 192.168.1.93`

### 问题 3: Git 凭据错误

**现象**: 无法克隆仓库

**解决**:
1. 检查 Jenkins 凭据配置中的用户名密码
2. 确认 GitLab 用户有仓库访问权限
3. 使用访问令牌代替密码

### 问题 4: 构建超时

**现象**: 构建运行时间过长被中断

**解决**:
在 Jenkinsfile 顶部添加:
```groovy
options {
    timeout(time: 2, unit: 'HOURS')
}
```

## 高级配置

### 多分支 Pipeline

如果需要同时监控多个分支:

1. 创建 "Multibranch Pipeline" 任务
2. 配置 Branch Sources 指向 Git 仓库
3. 设置 "Discover branches" 策略
4. Jenkinsfile 会自动应用到所有分支

### 并行构建

如果服务器资源充足，可以并行编译 x86 和 ARM:

```groovy
stage('并行架构编译') {
    parallel {
        stage('x86 编译') {
            steps {
                script {
                    // x86 编译流程
                }
            }
        }
        stage('ARM 编译') {
            steps {
                script {
                    // ARM 编译流程
                }
            }
        }
    }
}
```

### 构建产物归档

在 `post` 部分添加:

```groovy
always {
    archiveArtifacts artifacts: 'build_*_shared/**/log/*.log',
                     allowEmptyArchive: true
}
```

## 维护建议

1. **定期清理旧构建**: Jenkins 会根据配置自动清理
2. **监控磁盘空间**: 构建产物可能占用大量空间
3. **更新 Docker 镜像**: 定期更新构建镜像版本
4. **审查构建日志**: 定期检查警告和错误信息

## 相关文档

- [Jenkinsfile 语法](https://www.jenkins.io/doc/book/pipeline/syntax/)
- [项目 README](../README.md)
- [分层构建说明](../CLAUDE.md)
- [vcstool 团队指南](../VCSTOOL_TEAM_GUIDE.md)
