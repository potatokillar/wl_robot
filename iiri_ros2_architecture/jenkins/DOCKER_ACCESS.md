# Jenkins Docker 访问配置

## 问题说明

Jenkins 构建失败，错误信息：
```
docker: command not found
```

**原因**：Jenkins 用户无法访问 Docker 命令。

## 解决方案

### 场景 1: Jenkins 运行在宿主机上

如果 Jenkins 直接运行在宿主机上（非容器），需要将 Jenkins 用户添加到 docker 组：

```bash
# 1. 将 jenkins 用户添加到 docker 组
sudo usermod -aG docker jenkins

# 2. 重启 Jenkins 服务
sudo systemctl restart jenkins

# 3. 验证权限
sudo -u jenkins docker ps
```

### 场景 2: Jenkins 运行在 Docker 容器中

如果 Jenkins 运行在 Docker 容器中，需要挂载 Docker socket：

#### 方法 1: 重新创建容器（推荐）

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

#### 方法 2: 修改现有容器配置

```bash
# 1. 找到 Jenkins 容器 ID
docker ps | grep jenkins

# 2. 停止容器
docker stop <jenkins_container_id>

# 3. 使用 docker commit 保存当前状态
docker commit <jenkins_container_id> jenkins-with-docker

# 4. 重新运行新镜像，挂载 Docker socket
docker run -d \
    --name jenkins \
    -p 8080:8080 \
    -p 50000:50000 \
    -v jenkins_home:/var/jenkins_home \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /usr/bin/docker:/usr/bin/docker \
    jenkins-with-docker

# 5. 设置 Docker socket 权限
docker exec -it -u root jenkins bash
chmod 666 /var/run/docker.sock
# 或者将 jenkins 用户添加到 docker 组
groupadd -g $(stat -c '%g' /var/run/docker.sock) docker
usermod -aG docker jenkins
exit
```

#### 方法 3: 使用自定义 Dockerfile（最佳实践）

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

### 场景 3: 使用 sudo 运行 Docker（不推荐）

如果无法修改权限，可以配置 Jenkins 用 sudo 运行 docker：

1. 编辑 sudoers 文件：
```bash
sudo visudo
```

2. 添加以下行：
```
jenkins ALL=(ALL) NOPASSWD: /usr/bin/docker
```

3. 修改 `build_layered.sh` 脚本，将 `docker` 命令改为 `sudo docker`

## 验证安装

安装完成后，在 Jenkins 中重新触发构建：

1. 访问: http://192.168.1.93:8080/job/iiri-layered-build-ci
2. 点击 "Build with Parameters"
3. 选择参数并点击 "Build"

构建应该能够通过 Docker 检查并成功启动编译容器。

## 预期的构建流程

配置 Docker 访问后，Jenkins 构建将按以下流程执行：

```
1. 拉取主仓库 (master 分支)
2. 配置 Git 凭据
3. ✓ 检查 vcstool (已安装)
4. ✓ 检查 Docker (应该成功)
5. 使用 ./sync.sh import 导入所有分层代码
6. 使用 ./build_layered.sh 编译各层（在 Docker 容器中）:
   ├── Core Layer
   ├── Hardware Layer
   ├── Perception Layer
   ├── Intelligence Layer
   └── Application Layer
```

## 故障排查

### 问题：permission denied while trying to connect to the Docker daemon socket

**错误信息**:
```
Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock
```

**原因**: Jenkins 用户没有权限访问 Docker socket

**解决方法**:
```bash
# 在宿主机上
sudo chmod 666 /var/run/docker.sock

# 或者在容器中
docker exec -it -u root jenkins chmod 666 /var/run/docker.sock
```

### 问题：Cannot connect to the Docker daemon

**错误信息**:
```
Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?
```

**原因**: Docker socket 没有挂载到容器中

**解决方法**: 重新创建容器，添加 `-v /var/run/docker.sock:/var/run/docker.sock`

### 问题：docker: command not found (在容器中)

**错误信息**:
```
docker: command not found
```

**原因**: 容器中没有安装 Docker CLI

**解决方法**:
```bash
docker exec -it -u root jenkins bash
apt-get update
apt-get install -y docker.io
exit
```

### 问题：docker ps 返回权限错误

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

## 相关文档

- [Jenkins CI 快速开始](./QUICK_START.md)
- [vcstool 安装说明](./INSTALL_VCSTOOL.md)
- [详细配置指南](../docs/JENKINS_CI_SETUP.md)
