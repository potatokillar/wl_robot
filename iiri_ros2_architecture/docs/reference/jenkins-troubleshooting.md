# Jenkins CI 故障排除指南

> **文档位置**: `docs/reference/jenkins-troubleshooting.md`
> **最后更新**: 2025-10-13
> **作者**: 唐文浩

本文档提供 Jenkins CI 常见问题的诊断和解决方案。

## 🔍 故障排查流程

当 Jenkins 构建失败时，按以下顺序排查：

1. **查看构建日志** - 点击 Console Output 查看详细错误
2. **识别问题类型** - 根据错误信息定位到具体问题类别
3. **应用解决方案** - 参考本文档对应章节解决
4. **验证修复** - 重新触发构建确认问题已解决

## 📋 常见问题目录

- [vcstool 未安装](#vcstool-未安装)
- [Docker 访问问题](#docker-访问问题)
- [Git 凭据错误](#git-凭据错误)
- [构建超时](#构建超时)
- [编译失败](#编译失败)
- [网络连接问题](#网络连接问题)

---

## vcstool 未安装

### 问题症状

```
vcstool 未安装在 Jenkins 服务器上
vcs: command not found
```

### 问题原因

vcstool 需要在 Jenkins 宿主机（或 Jenkins Docker 容器）中安装，用于拉取分层代码。

### 解决方案

#### 方案 1: 在 Jenkins 容器中安装（推荐）

如果 Jenkins 运行在 Docker 容器中：

```bash
# 1. 找到 Jenkins 容器 ID
docker ps | grep jenkins

# 2. 进入 Jenkins 容器
docker exec -it -u root <jenkins_container_id> bash

# 3. 在容器中安装 vcstool
apt-get update
apt-get install -y python3-pip
pip3 install vcstool --break-system-packages

# 4. 验证安装
vcs --version

# 5. 退出容器
exit
```

**注意**: `--break-system-packages` 在容器环境中是安全的，因为容器是隔离的。

#### 方案 2: 在 Jenkins Docker 镜像中预装

创建自定义 Dockerfile：

```dockerfile
FROM jenkins/jenkins:lts

USER root

# 安装 vcstool
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install vcstool --break-system-packages && \
    rm -rf /var/lib/apt/lists/*

USER jenkins
```

构建并运行：

```bash
# 构建自定义镜像
docker build -t jenkins-with-vcstool .

# 运行新镜像
docker run -d \
    -p 8080:8080 \
    -v jenkins_home:/var/jenkins_home \
    --name jenkins \
    jenkins-with-vcstool
```

#### 方案 3: 在宿主机安装

如果 Jenkins 直接运行在宿主机上：

```bash
# 使用 pip 安装（推荐）
sudo apt-get update
sudo apt-get install -y python3-pip
sudo pip3 install vcstool

# 或者使用 apt（如果软件源支持）
sudo apt-get install -y python3-vcstool

# 验证安装
vcs --version
```

### 常见子问题

#### 错误：externally-managed-environment

**错误信息**:
```
error: externally-managed-environment
× This environment is externally managed
```

**原因**: Python 3.11+ 使用 PEP 668 防止 pip 破坏系统包管理。

**解决方法**:
```bash
pip3 install vcstool --break-system-packages
```

#### 错误：权限不足

**解决方法**:
- 使用 `root` 用户进入容器: `docker exec -it -u root ...`
- 或使用 `sudo` 安装

#### 错误：安装后仍然提示未找到

检查 PATH 环境变量：

```bash
which vcs
echo $PATH
```

确保 vcstool 安装路径在 PATH 中。

---

## Docker 访问问题

### 问题症状

```
docker: command not found
```

或

```
Got permission denied while trying to connect to the Docker daemon socket
```

### 问题原因

Jenkins 用户无法访问 Docker 命令或没有权限访问 Docker socket。

### 解决方案

#### 场景 1: Jenkins 运行在宿主机上

将 Jenkins 用户添加到 docker 组：

```bash
# 1. 将 jenkins 用户添加到 docker 组
sudo usermod -aG docker jenkins

# 2. 重启 Jenkins 服务
sudo systemctl restart jenkins

# 3. 验证权限
sudo -u jenkins docker ps
```

#### 场景 2: Jenkins 运行在 Docker 容器中

需要挂载 Docker socket：

**方法 1: 重新创建容器（推荐）**

```bash
# 停止并删除现有容器
docker stop jenkins
docker rm jenkins

# 重新运行，挂载 Docker socket
docker run -d \
    --name jenkins \
    -p 8080:8080 \
    -p 50000:50000 \
    -v jenkins_home:/var/jenkins_home \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /usr/bin/docker:/usr/bin/docker \
    jenkins/jenkins:lts

# 进入容器安装 Docker CLI（如果需要）
docker exec -it -u root jenkins bash
apt-get update
apt-get install -y docker.io
exit
```

**方法 2: 使用自定义 Dockerfile（最佳实践）**

创建 `Dockerfile.jenkins-docker`:

```dockerfile
FROM jenkins/jenkins:lts

USER root

# 安装 Docker CLI
RUN apt-get update && \
    apt-get install -y docker.io && \
    rm -rf /var/lib/apt/lists/*

# 安装 vcstool
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install vcstool --break-system-packages && \
    rm -rf /var/lib/apt/lists/*

USER jenkins
```

构建并运行：

```bash
# 构建镜像
docker build -t jenkins-with-docker -f Dockerfile.jenkins-docker .

# 运行容器
docker run -d \
    --name jenkins \
    -p 8080:8080 \
    -p 50000:50000 \
    -v jenkins_home:/var/jenkins_home \
    -v /var/run/docker.sock:/var/run/docker.sock \
    jenkins-with-docker
```

#### 场景 3: 使用 sudo 运行 Docker（不推荐）

1. 编辑 sudoers 文件：
```bash
sudo visudo
```

2. 添加以下行：
```
jenkins ALL=(ALL) NOPASSWD: /usr/bin/docker
```

### 常见子问题

#### 错误：permission denied while trying to connect to Docker socket

**错误信息**:
```
Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock
```

**解决方法**:
```bash
# 在宿主机上
sudo chmod 666 /var/run/docker.sock

# 或者在容器中
docker exec -it -u root jenkins chmod 666 /var/run/docker.sock

# 更好的方式：将 jenkins 用户添加到 docker 组
docker exec -it -u root jenkins bash
groupadd -g $(stat -c '%g' /var/run/docker.sock) docker
usermod -aG docker jenkins
exit
```

#### 错误：Cannot connect to the Docker daemon

**错误信息**:
```
Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?
```

**原因**: Docker socket 没有挂载到容器中

**解决方法**: 重新创建容器，添加 `-v /var/run/docker.sock:/var/run/docker.sock`

#### 错误：docker: command not found (在容器中)

**原因**: 容器中没有安装 Docker CLI

**解决方法**:
```bash
docker exec -it -u root jenkins bash
apt-get update
apt-get install -y docker.io
exit
```

#### 验证 Docker 权限

检查 Docker socket 权限：
```bash
ls -la /var/run/docker.sock
# 应该显示 srw-rw---- 或 srw-rw-rw-
```

检查 Jenkins 用户是否在 docker 组中：
```bash
groups jenkins
# 应该包含 "docker"
```

---

## Git 凭据错误

### 问题症状

```
fatal: Authentication failed
fatal: Could not read from remote repository
```

### 问题原因

Jenkins 无法访问 Git 仓库，凭据配置错误或过期。

### 解决方案

#### 检查凭据配置

1. 进入 Jenkins 首页
2. 点击 "Manage Jenkins" → "Manage Credentials"
3. 检查凭据 ID 为 `git-cred` 是否存在
4. 验证用户名和密码是否正确

#### 重新配置凭据

1. 删除旧凭据（如果存在）
2. 创建新凭据：
   - **Kind**: Username with password
   - **Username**: `root` 或你的 GitLab 用户名
   - **Password**: 你的 GitLab 密码或访问令牌
   - **ID**: `git-cred`
   - **Description**: GitLab credentials

#### 使用访问令牌（推荐）

在 GitLab 中生成个人访问令牌：

1. 登录 GitLab
2. 进入 Settings → Access Tokens
3. 创建新令牌，权限选择：`read_repository`, `write_repository`
4. 在 Jenkins 凭据中使用令牌作为密码

---

## 构建超时

### 问题症状

```
Build timed out (after N minutes)
```

### 问题原因

编译时间过长超过 Jenkins 默认超时限制。

### 解决方案

在 Jenkinsfile 顶部添加超时配置：

```groovy
pipeline {
    agent any

    options {
        timeout(time: 2, unit: 'HOURS')
    }

    // ... 其他配置
}
```

或在任务配置中设置：

1. 进入任务配置页面
2. 在 "Build Environment" 中勾选 "Abort the build if it's stuck"
3. 设置合适的超时时间

---

## 编译失败

### 问题症状

```
CMake Error
colcon build failed
Package '...' failed
```

### 排查步骤

#### 1. 查看详细日志

```bash
# 在构建日志中找到失败的层
# 查看对应的 log 文件路径
```

#### 2. 本地复现

```bash
# 在本地执行相同的构建命令
cd /path/to/workspace
./build_layered.sh -c <failed_layer>
```

#### 3. 检查依赖

```bash
# 检查 ROS2 依赖
rosdep check --from-paths src --ignore-src

# 安装缺失依赖
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. 清理构建

```bash
# 清理失败层的构建产物
rm -rf build_*/failed_layer/

# 重新构建
./build_layered.sh -c failed_layer
```

---

## 网络连接问题

### 问题症状

```
Could not resolve host
Connection timed out
Failed to fetch
```

### 解决方案

#### 检查网络连通性

```bash
# 测试 Git 服务器连接
ping 192.168.1.55

# 测试 Harbor 连接
ping 192.168.1.93

# 测试 DNS
nslookup google.com
```

#### 配置 HTTP 代理

如果需要通过代理访问外网：

在 Jenkinsfile 中添加：

```groovy
environment {
    HTTP_PROXY = 'http://proxy-server:port'
    HTTPS_PROXY = 'http://proxy-server:port'
    NO_PROXY = 'localhost,127.0.0.1,192.168.1.*'
}
```

#### Docker 镜像拉取失败

```bash
# 检查 Harbor 可访问性
curl http://192.168.1.93

# 手动拉取镜像测试
docker pull 192.168.1.93/iiri/build_x86_ros2:v1.4.3

# 检查 Docker 配置
cat /etc/docker/daemon.json
```

---

## 🛠️ 调试技巧

### 1. 在 Jenkins 中执行 Shell 脚本调试

在 Jenkinsfile 中添加调试信息：

```groovy
sh '''
    set -x  # 打印执行的命令
    echo "当前目录: $(pwd)"
    echo "用户: $(whoami)"
    echo "环境变量:"
    env | sort

    # 你的构建命令
    ./build_layered.sh
'''
```

### 2. 保留工作空间

修改 Jenkinsfile，注释掉清理步骤：

```groovy
// post {
//     always {
//         cleanWs()
//     }
// }
```

然后进入 Jenkins 工作空间手动调试：

```bash
cd /var/jenkins_home/workspace/iiri-layered-build-ci
ls -la
```

### 3. 使用 Jenkins Script Console

1. 进入 Jenkins → Manage Jenkins → Script Console
2. 执行 Groovy 脚本查看系统状态：

```groovy
// 查看环境变量
println System.getenv()

// 查看所有任务
Jenkins.instance.getAllItems().each { println it.name }
```

---

## 📞 获取帮助

### 优先级排查顺序

1. **查看构建日志** - 90% 的问题可以从日志中找到线索
2. **参考本文档** - 查找对应的问题类型和解决方案
3. **本地复现** - 在本地环境尝试复现问题
4. **检查配置** - 验证 Jenkins、Git、Docker 配置
5. **联系团队** - 如果以上步骤都无法解决，联系技术支持

### 相关文档

- **[Jenkins 配置指南](../ci-cd/jenkins-setup.md)** - 初始配置说明
- **[Jenkins 使用指南](../ci-cd/jenkins-usage.md)** - 日常操作方法
- **[构建指南](../development/build-guide.md)** - 本地构建说明
- **[部署指南](../deployment/deployment-guide.md)** - 部署流程

### 社区资源

- [Jenkins 官方文档](https://www.jenkins.io/doc/)
- [Docker 官方文档](https://docs.docker.com/)
- [vcstool 官方仓库](https://github.com/dirk-thomas/vcstool)

---

**最后更新**: 2025-10-13
**版本**: v1.0
**作者**: 唐文浩
