# 在 Jenkins 服务器上安装 vcstool

## 问题说明

Jenkins 构建失败，错误信息：
```
vcstool 未安装在 Jenkins 服务器上
```

**原因**：vcstool 需要在 Jenkins 宿主机（或 Jenkins Docker 容器）中安装，用于拉取分层代码。

## 解决方案

### 方案 1: 在 Jenkins 容器中安装（推荐）

如果 Jenkins 运行在 Docker 容器中，需要进入容器安装：

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

### 方案 2: 在 Jenkins Docker 镜像中预装

更好的方式是在 Jenkins Docker 镜像中预装 vcstool：

创建自定义 Dockerfile:

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

构建并运行:

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

### 方案 3: 在宿主机安装（如果 Jenkins 直接运行在宿主机）

如果 Jenkins 直接运行在宿主机上：

```bash
# Ubuntu/Debian - 使用 pip 安装（推荐）
sudo apt-get update
sudo apt-get install -y python3-pip
sudo pip3 install vcstool

# 或者使用 apt（如果软件源支持）
sudo apt-get install -y python3-vcstool

# 验证安装
vcs --version
```

## 验证安装

安装完成后，在 Jenkins 中重新触发构建：

1. 访问: http://192.168.1.93:8080/job/iiri-layered-build-ci
2. 点击 "Build with Parameters"
3. 选择参数并点击 "Build"

构建应该能够通过 vcstool 检查并成功导入分层代码。

## 预期的构建流程

安装 vcstool 后，Jenkins 构建将按以下流程执行：

```
1. 拉取主仓库 (master 分支)
2. 配置 Git 凭据
3. ✓ 检查 vcstool (应该成功)
4. 使用 ./sync.sh import 导入所有分层代码
5. 使用 ./build_layered.sh 编译各层（在 Docker 容器中）
```

## 故障排查

### 问题：externally-managed-environment 错误

**错误信息**:
```
error: externally-managed-environment
× This environment is externally managed
```

**原因**: Python 3.11+ 使用 PEP 668 防止 pip 破坏系统包管理。

**解决方法**: 在 Jenkins 容器中使用 `--break-system-packages` 是安全的：
```bash
pip3 install vcstool --break-system-packages
```

这在容器环境中是安全的，因为容器是隔离的，不会影响宿主机。

### 问题：权限不足

如果出现权限错误，确保：
- 使用 `root` 用户进入容器: `docker exec -it -u root ...`
- 或使用 `sudo` 安装

### 问题：apt-get 失败

如果 apt-get 失败：
```bash
# 更新软件源
apt-get update

# 如果网络问题，配置代理
export http_proxy=http://your-proxy:port
export https_proxy=http://your-proxy:port
```

### 问题：安装后仍然提示未找到

检查 PATH 环境变量：
```bash
which vcs
echo $PATH
```

确保 vcstool 安装路径在 PATH 中。

## 相关文档

- [Jenkins CI 快速开始](./QUICK_START.md)
- [详细配置指南](../docs/JENKINS_CI_SETUP.md)
- [vcstool 官方文档](https://github.com/dirk-thomas/vcstool)
